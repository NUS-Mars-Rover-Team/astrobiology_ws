/*
  Six independent ESP32-WROOM-32 + TMC2208 stepper channels.
  Hardware Timer Interrupt Driven - Capable of jitter-free 100kHz+ steps.
  Zero external stepper libraries required.
  Serial: 115200, Newline.

  Custom G1 syntax (NOT standard CNC G-code), decimals allowed:
    g1 <id> <distance> <speed> <accel>
    g1 x 10.5 5 2        x: move +10.5 mm at 5 mm/s, accel 2 mm/s^2
    g1 c 2 120 60        centrifuge: 2 rev at 120 RPM, accel 60 RPM/s
    g1 p 3200 800 400    pump/flush: raw pulses, pulses/s, pulses/s^2

  Units (conversion ratios are hard-coded in the UNITS section below):
    x y z : mm, mm/s, mm/s^2     (200 steps/rev x 16 microstep / 1 mm pitch)
    c     : rev, RPM, RPM/s      (200 steps/rev x 8 microstep)
    p f   : pulses, pulses/s, pulses/s^2

  Coordinates:  pos  (or m114)  ->  pos X:.. Y:.. Z:.. C:.. P:.. F:..

  Homing (G28 style):
    g28          home all axes that have an endstop (order: HOME_ORDER below)
    g28 x        home X only
    g28 x y      home X then Y (always runs in HOME_ORDER, one axis at a time)

  ESP32 NOTE: GPIO34/35 are input-only and have NO internal pull-up.
  If your endstop switches to GND, add an external 10k pull-up to 3.3V on
  those pins (or flip ENDSTOP_TRIGGER_LEVEL for a switch that pulls to 3.3V).
*/

#include <Arduino.h>
#include "soc/gpio_reg.h" // ESP32 Hardware Register Definitions
#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// ==================================================================
//                        USER CONFIGURATION
// ==================================================================

// ---- MOTOR DIRECTION --------------------------------------------
// false = positive steps drive DIR pin HIGH
// true  = positive steps drive DIR pin LOW  (flips the motor's physical direction)
const bool INVERT_DIR_Z = true;
const bool INVERT_DIR_Y = false;
const bool INVERT_DIR_X = false;
const bool INVERT_DIR_C = false;   // centrifuge
const bool INVERT_DIR_P = false;   // pump
const bool INVERT_DIR_F = false;   // flush

// ---- UNITS / CONVERSION RATIOS (hard-coded) ----------------------
// steps per unit = full steps per rev * microsteps / (mm per rev for linear axes)
//   x y z : unit = mm   (200 steps/rev, 1/16 microstep, 1 mm pitch) = 3200 steps/mm
//   c     : unit = rev  (200 steps/rev, 1/8 microstep)              = 1600 steps/rev
//   p f   : unit = raw pulses (1:1)
const double STEPS_PER_UNIT_X = 200.0 * 16.0 / 1.0;
const double STEPS_PER_UNIT_Y = 200.0 * 16.0 / 1.0;
const double STEPS_PER_UNIT_Z = 200.0 * 16.0 / 1.0;
const double STEPS_PER_UNIT_C = 200.0 * 8.0;
const double STEPS_PER_UNIT_P = 1.0;
const double STEPS_PER_UNIT_F = 1.0;

// Time base of speed and accel: 1 = units/s (and units/s^2),
// 60 = units/min (and units/min per second), i.e. RPM and RPM/s for the centrifuge.
const double TIME_DIV_X = 1.0;
const double TIME_DIV_Y = 1.0;
const double TIME_DIV_Z = 1.0;
const double TIME_DIV_C = 60.0;
const double TIME_DIV_P = 1.0;
const double TIME_DIV_F = 1.0;

// ---- ENDSTOP PINS -----------------------------------------------
const uint8_t ENDSTOP_X_PIN = 35;
const uint8_t ENDSTOP_Y_PIN = 34;
const uint8_t ENDSTOP_Z_PIN = 21;

// Pin level when the endstop is PRESSED.
//   LOW  = switch connects pin to GND (pulled up otherwise)
//   HIGH = switch connects pin to 3.3V
const uint8_t ENDSTOP_TRIGGER_LEVEL = HIGH;

// Number of consecutive 4us ISR samples the pin must be triggered before it
// counts (simple debounce / noise filter). 8 = 32us.
const uint8_t ENDSTOP_DEBOUNCE_TICKS = 8;

// ---- HOMING DIRECTION (sign of the steps that move TOWARD the endstop) ----
const int8_t HOME_DIR_X = -1;
const int8_t HOME_DIR_Y = -1;
const int8_t HOME_DIR_Z = -1;

// ---- HOMING PARAMETERS (per axis) -------------------------------
// !! These are starting guesses - tune them for your machine !!
// (Homing values are in RAW STEPS, not mm - they are internal settings.)
//   FAST   : search speed (steps/s) until the endstop is found
//   SLOW   : final precise touch speed (steps/s)
//   ACCEL  : acceleration used during homing (steps/s^2)
//   BACKOFF: steps to retreat after the first hit (must be enough to release)
//   MAXTRAVEL: give up if the endstop isn't found within this many steps
const uint32_t HOME_FAST_X = 10000, HOME_SLOW_X = 5000, HOME_ACCEL_X = 5000;
const int32_t  HOME_BACKOFF_X = 8000, HOME_MAXTRAVEL_X = 1000000;

const uint32_t HOME_FAST_Y = 10000,  HOME_SLOW_Y = 5000,  HOME_ACCEL_Y = 5000;
const int32_t  HOME_BACKOFF_Y = 8000, HOME_MAXTRAVEL_Y = 1000000;

const uint32_t HOME_FAST_Z = 10000, HOME_SLOW_Z = 5000, HOME_ACCEL_Z = 5000;
const int32_t  HOME_BACKOFF_Z = 8000, HOME_MAXTRAVEL_Z = 1000000;

// ==================================================================

const uint32_t MAX_SPEED_HZ = 100000;
const uint32_t MAX_ACCEL = 1000000;
const uint32_t ENABLE_SETTLE_MS = 2;
const size_t LINE_SIZE = 160;
const uint8_t NO_ENDSTOP = 0xFF;

// Timer configuration: 4 us ISR period (250 kHz).
// Step timing is computed from the CPU cycle counter, NOT from the tick count,
// so motor speed stays correct even if the ISR can't quite keep up with the
// timer. Max achievable step rate is roughly (real ISR rate / 2); use `diag`.
const uint32_t ISR_PERIOD_US = 4;
const uint32_t ISR_FREQ = 1000000UL / ISR_PERIOD_US;

uint32_t cpuHz = 240000000UL;      // set in setup() from the real CPU clock
uint32_t maxDtCycles = 24000;      // clamp for one ISR interval (set in setup)
uint32_t isrLastCycles = 0;
volatile uint32_t isrTicks = 0;
volatile uint32_t isrMaxCycles = 0;
hw_timer_t * stepTimer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

enum MotionState { IDLE, WAIT_ENABLE, MOVING };
enum HomePhase { HOME_NONE, HOME_PRE_BACKOFF, HOME_FAST, HOME_BACKOFF, HOME_SLOW };

struct Motor {
  const char *name;
  const char *shortName;
  uint8_t stepPin;
  uint8_t dirPin;
  uint8_t enPin;
  bool positiveDirHigh;

  // Units
  double stepsPerUnit;
  double timeDiv;
  const char *unitLabel;
  uint8_t posDecimals;

  // Homing configuration
  uint8_t endPin;
  int8_t homeDir;
  uint32_t homeFastHz;
  uint32_t homeSlowHz;
  uint32_t homeAccel;
  int32_t homeBackoff;
  int32_t homeMaxTravel;

  bool enabled;
  bool disableAfterStop;
  uint32_t enabledAt;

  // Kinematics (Managed by main loop)
  uint32_t targetSpeedHz;
  uint32_t accelHz2;
  float currentSpeedHz;
  uint32_t lastCalcUs;
  bool stopRequested;
  bool stopRetargeted;

  // Homing state (main loop only)
  HomePhase homePhase;
  bool homeRequested;
  uint8_t homeRetries;

  // Real-time variables (Shared with ISR - must be volatile)
  volatile MotionState state;
  volatile int32_t currentPos;
  volatile int32_t targetPos;
  volatile uint32_t isrSpeed;
  volatile uint32_t accumulator;
  volatile int8_t moveDir;
  volatile bool stepPinHigh;
  volatile bool watchEndstop;
  volatile bool endstopHit;
  volatile uint8_t endCount;

  Motor(const char *n, const char *sn, uint8_t st, uint8_t di, uint8_t en,
        bool invertDir, double spu, double tdiv, const char *label,
        uint8_t endstop = NO_ENDSTOP, int8_t hDir = -1,
        uint32_t hFast = 0, uint32_t hSlow = 0, uint32_t hAccel = 0,
        int32_t hBackoff = 0, int32_t hMax = 0)
      : name(n), shortName(sn), stepPin(st), dirPin(di), enPin(en),
        positiveDirHigh(!invertDir),
        stepsPerUnit(spu), timeDiv(tdiv), unitLabel(label), posDecimals(spu > 1.0 ? 4 : 0),
        endPin(endstop), homeDir(hDir), homeFastHz(hFast), homeSlowHz(hSlow),
        homeAccel(hAccel), homeBackoff(hBackoff), homeMaxTravel(hMax),
        enabled(false), disableAfterStop(false),
        enabledAt(0), targetSpeedHz(1000), accelHz2(500), currentSpeedHz(0.0f),
        lastCalcUs(0), stopRequested(false), stopRetargeted(false),
        homePhase(HOME_NONE), homeRequested(false), homeRetries(0),
        state(IDLE), currentPos(0),
        targetPos(0), isrSpeed(0), accumulator(0), moveDir(1), stepPinHigh(false),
        watchEndstop(false), endstopHit(false), endCount(0) {}
};

//                 full  short       STEP DIR EN   invert dir, steps/unit, time div, unit label,  endstop pin       home dir     fast         slow         accel         backoff         maxTravel
Motor motors[] = {
  Motor("z", "z",          2,  4, 15, INVERT_DIR_Z, STEPS_PER_UNIT_Z, TIME_DIV_Z, "mm", ENDSTOP_Z_PIN, HOME_DIR_Z, HOME_FAST_Z, HOME_SLOW_Z, HOME_ACCEL_Z, HOME_BACKOFF_Z, HOME_MAXTRAVEL_Z),
  Motor("y", "y",         16, 17, 15, INVERT_DIR_Y, STEPS_PER_UNIT_Y, TIME_DIV_Y, "mm", ENDSTOP_Y_PIN, HOME_DIR_Y, HOME_FAST_Y, HOME_SLOW_Y, HOME_ACCEL_Y, HOME_BACKOFF_Y, HOME_MAXTRAVEL_Y),
  Motor("x", "x",         18, 19, 15, INVERT_DIR_X, STEPS_PER_UNIT_X, TIME_DIV_X, "mm", ENDSTOP_X_PIN, HOME_DIR_X, HOME_FAST_X, HOME_SLOW_X, HOME_ACCEL_X, HOME_BACKOFF_X, HOME_MAXTRAVEL_X),
  Motor("c", "centrifuge",32, 33,  5, INVERT_DIR_C, STEPS_PER_UNIT_C, TIME_DIV_C, "rev"),
  Motor("p", "pump",      26, 27, 25, INVERT_DIR_P, STEPS_PER_UNIT_P, TIME_DIV_P, "pulses"),
  Motor("f", "flush",     12, 13, 14, INVERT_DIR_F, STEPS_PER_UNIT_F, TIME_DIV_F, "pulses")
};

const size_t MOTOR_COUNT = sizeof(motors) / sizeof(motors[0]);

// Indices into motors[] (keep in sync with the table above)
enum { M_Z = 0, M_Y = 1, M_X = 2, M_C = 3, M_P = 4, M_F = 5 };

// Order in which axes are homed when several are requested (e.g. "g28").
// Z first (lift clear), then Y, then X. Edit to suit your machine.
const uint8_t HOME_ORDER[] = { M_Z, M_Y, M_X };
const size_t HOME_ORDER_COUNT = sizeof(HOME_ORDER) / sizeof(HOME_ORDER[0]);

bool homeBatchActive = false;

char inputLine[LINE_SIZE];
size_t inputLength = 0;
bool discardLine = false;

// Function prototypes
void printHelp();
int findMotor(const char *name);
bool isBusy(const Motor &m);
bool parseNumber(const char *s, double &value);
bool unitsToSteps(const Motor &m, double units, int32_t &steps);
bool unitsToRate(const Motor &m, double units, uint32_t maximum, uint32_t &rate);
void printG1Limits(const Motor &m);
void servicePosReply();
void syncHardwareEnablePins();
bool enableMotor(Motor &m);
void disableMotor(Motor &m);
void reportEnd(const Motor &m, const char *event);
void requestStop(Motor &m, bool release);
void emergencyStop(Motor &m);
bool beginMove(Motor &m, int32_t distance, uint32_t speed, uint32_t accel, bool watchEndstop);
void startMove(Motor &m, int32_t distance, uint32_t speed, uint32_t accel);
void serviceMotors();
void printStatus(const Motor &m);
void handleHome(char **axes, size_t n);
void handleLine(char *line);
void readSerial();
void serviceHoming();
void serviceHomeQueue();
void printDiag();
bool endstopTriggered(const Motor &m);
void abortHoming(Motor &m, const __FlashStringHelper *reason);
bool homeMove(Motor &m, bool towardEndstop, int32_t steps, uint32_t speed, bool watch);
void startHoming(Motor &m);

// Fast GPIO register access bypasses the latency of digitalWrite()/digitalRead().
// Forced inline to avoid the Xtensa "dangerous relocation" l32r linker bug.
inline __attribute__((always_inline)) void fastDigitalWrite(uint8_t pin, bool val) {
  if (pin < 32) {
    if (val) REG_WRITE(GPIO_OUT_W1TS_REG, 1UL << pin);
    else     REG_WRITE(GPIO_OUT_W1TC_REG, 1UL << pin);
  } else {
    if (val) REG_WRITE(GPIO_OUT1_W1TS_REG, 1UL << (pin - 32));
    else     REG_WRITE(GPIO_OUT1_W1TC_REG, 1UL << (pin - 32));
  }
}

inline __attribute__((always_inline)) bool fastDigitalRead(uint8_t pin) {
  if (pin < 32) return (REG_READ(GPIO_IN_REG) >> pin) & 1U;
  return (REG_READ(GPIO_IN1_REG) >> (pin - 32)) & 1U;
}

// CPU cycle counter (always inline so it is safe inside the ISR)
inline __attribute__((always_inline)) uint32_t cycleCount() {
#if defined(ARDUINO_ARCH_ESP32) && defined(__XTENSA__)
  uint32_t c;
  __asm__ __volatile__("rsr %0, ccount" : "=a"(c));
  return c;
#else
  return static_cast<uint32_t>(micros() * (cpuHz / 1000000UL));
#endif
}

// ------------------------------------------------------------------
// HARDWARE TIMER INTERRUPT (nominally 250,000 times per second)
// Runs on the same core as loop(); loop() masks it with
// portENTER_CRITICAL when it touches shared variables, so no spinlock
// is needed here (keeps the ISR short).
// ------------------------------------------------------------------
void IRAM_ATTR onTimer() {
  const uint32_t t0 = cycleCount();
  uint32_t dt = t0 - isrLastCycles;        // real time since last tick, in CPU cycles
  isrLastCycles = t0;
  if (dt > maxDtCycles) dt = maxDtCycles;
  const uint32_t hz = cpuHz;

  for (size_t i = 0; i < MOTOR_COUNT; ++i) {
    Motor *m = &motors[i];

    // End the step pulse from the previous tick
    const bool wasHigh = m->stepPinHigh;
    if (wasHigh) {
      fastDigitalWrite(m->stepPin, false);
      m->stepPinHigh = false;
    }

    if (m->state != MOVING) continue;

    // Endstop watch (homing): halt instantly when the switch is hit
    if (m->watchEndstop) {
      if ((fastDigitalRead(m->endPin) ? HIGH : LOW) == ENDSTOP_TRIGGER_LEVEL) {
        if (m->endCount < 255) m->endCount = m->endCount + 1;
      } else {
        m->endCount = 0;
      }
      if (m->endCount >= ENDSTOP_DEBOUNCE_TICKS) {
        m->targetPos = m->currentPos;   // no more steps will be emitted
        m->endstopHit = true;
        m->watchEndstop = false;
      }
    }

    // Time-based phase accumulator: acc / hz = steps owed
    uint32_t acc = m->accumulator + m->isrSpeed * dt;
    if (acc >= hz && !wasHigh) {
      acc -= hz;
      if (m->currentPos != m->targetPos) {
        m->currentPos += m->moveDir;
        fastDigitalWrite(m->stepPin, true);
        m->stepPinHigh = true;
      }
    }
    if (acc > hz) acc = hz;                // never build a burst backlog
    m->accumulator = acc;
  }

  const uint32_t used = cycleCount() - t0;
  if (used > isrMaxCycles) isrMaxCycles = used;
  isrTicks = isrTicks + 1;
}

void syncHardwareEnablePins() {
  for (size_t i = 0; i < MOTOR_COUNT; ++i) {
    uint8_t pin = motors[i].enPin;
    bool anyEnabledOnThisPin = false;
    for (size_t j = 0; j < MOTOR_COUNT; ++j) {
      if (motors[j].enPin == pin && motors[j].enabled) {
        anyEnabledOnThisPin = true;
        break;
      }
    }
    digitalWrite(pin, anyEnabledOnThisPin ? LOW : HIGH);
  }
}

void printHelp() {
  Serial.println(F("Custom commands; IDs: pump(p) centrifuge(c) flush(f) x y z"));
  Serial.println(F("g1 <id> <distance> <speed> <accel>   (decimals ok)"));
  Serial.println(F("  x y z: mm, mm/s, mm/s^2   c: rev, RPM, RPM/s   p f: pulses, pulses/s, pulses/s^2"));
  Serial.println(F("Example: g1 x 10.5 5 2   (10.5 mm at 5 mm/s, accel 2 mm/s^2)"));
  Serial.println(F("pos | m114   print all coordinates (non-blocking, safe to poll fast)"));
  Serial.println(F("G1 is relative, auto-enables, and holds after the move."));
  Serial.println(F("g28 [x] [y] [z]   home listed axes (no args = all); sets pos to 0"));
  Serial.println(F("enable <id|all> on|off   stop <id|all> (decelerates, then halts)"));
  Serial.println(F("estop [id|all]   status [id|all]   zero <id|all>   diag   help"));
  Serial.println(F("Send '!' (no Enter needed) for an instant emergency stop of everything."));
  Serial.println(F("Internally speed is limited to 1..100000 steps/s, accel to 1..1000000 steps/s^2."));
  Serial.println(F("Wait for done before another G1 on the same motor."));
}

int findMotor(const char *name) {
  for (size_t i = 0; i < MOTOR_COUNT; ++i) {
    if (strcmp(name, motors[i].name) == 0 ||
        strcmp(name, motors[i].shortName) == 0) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

bool isBusy(const Motor &m) {
  return m.state != IDLE || m.homePhase != HOME_NONE || m.homeRequested;
}

bool endstopTriggered(const Motor &m) {
  return digitalRead(m.endPin) == ENDSTOP_TRIGGER_LEVEL;
}

// Plain decimal number: optional sign, digits, optional single '.', no exponent
bool parseNumber(const char *s, double &value) {
  const char *p = s;
  if (*p == '+' || *p == '-') ++p;
  bool digits = false, dot = false;
  for (; *p; ++p) {
    if (*p >= '0' && *p <= '9') digits = true;
    else if (*p == '.' && !dot) dot = true;
    else return false;
  }
  if (!digits) return false;
  value = strtod(s, nullptr);
  return true;
}

// Signed distance in motor units -> steps
bool unitsToSteps(const Motor &m, double units, int32_t &steps) {
  const double v = units * m.stepsPerUnit;
  if (v >= 2147483647.0 || v <= -2147483647.0) return false;
  steps = static_cast<int32_t>(llround(v));
  return true;
}

// Speed or accel (units/time) -> steps/s or steps/s^2, must be 1..maximum
bool unitsToRate(const Motor &m, double units, uint32_t maximum, uint32_t &rate) {
  if (!(units > 0.0)) return false;
  const double v = units * m.stepsPerUnit / m.timeDiv;
  if (v > static_cast<double>(maximum) + 0.5) return false;
  const long long r = llround(v);
  if (r < 1 || r > static_cast<long long>(maximum)) return false;
  rate = static_cast<uint32_t>(r);
  return true;
}

void printG1Limits(const Motor &m) {
  const bool perMin = (m.timeDiv > 1.0);
  Serial.print(F("error bad number for ")); Serial.print(m.name);
  Serial.print(F(": distance in ")); Serial.print(m.unitLabel);
  Serial.print(F(", speed 0..")); Serial.print(static_cast<double>(MAX_SPEED_HZ) * m.timeDiv / m.stepsPerUnit, 3);
  Serial.print(' '); Serial.print(m.unitLabel); Serial.print(perMin ? F("/min") : F("/s"));
  Serial.print(F(", accel 0..")); Serial.print(static_cast<double>(MAX_ACCEL) * m.timeDiv / m.stepsPerUnit, 3);
  Serial.print(' '); Serial.print(m.unitLabel); Serial.println(perMin ? F("/min/s") : F("/s^2"));
}

bool enableMotor(Motor &m) {
  if (!m.enabled) {
    m.enabled = true;
    m.enabledAt = millis();
    syncHardwareEnablePins();
  }
  return true;
}

void disableMotor(Motor &m) {
  m.enabled = false;
  m.disableAfterStop = false;
  syncHardwareEnablePins();
}

void reportEnd(const Motor &m, const char *event) {
  Serial.print(event); Serial.print(' '); Serial.print(m.name);
  Serial.print(F(" steps=")); Serial.print(m.currentPos);
  Serial.print(F(" pos=")); Serial.println(static_cast<double>(m.currentPos) / m.stepsPerUnit, m.posDecimals);
}

void requestStop(Motor &m, bool release) {
  // Any stop cancels homing on this axis (and its pending queue entry)
  m.homeRequested = false;
  m.homePhase = HOME_NONE;
  m.watchEndstop = false;

  m.disableAfterStop = m.disableAfterStop || release;
  if (m.state == WAIT_ENABLE) {
    m.state = IDLE;
    reportEnd(m, "cancelled");
    if (m.disableAfterStop) disableMotor(m);
    return;
  }
  if (m.state == MOVING) {
    m.stopRequested = true;
  } else if (m.state == IDLE) {
    if (m.disableAfterStop) disableMotor(m);
  }
}

void emergencyStop(Motor &m) {
  portENTER_CRITICAL(&timerMux);
  m.state = IDLE;
  m.isrSpeed = 0;
  m.watchEndstop = false;
  m.targetPos = m.currentPos;
  fastDigitalWrite(m.stepPin, false);
  m.stepPinHigh = false;
  portEXIT_CRITICAL(&timerMux);

  m.homePhase = HOME_NONE;
  m.homeRequested = false;
  m.currentSpeedHz = 0.0f;
  m.disableAfterStop = false;
  m.enabled = false;
  syncHardwareEnablePins();
}

// Low-level move start (no serial output). Returns false if it can't start.
bool beginMove(Motor &m, int32_t distance, uint32_t speed, uint32_t accel, bool watch) {
  if (m.state != IDLE || distance == 0) return false;
  const int64_t target = static_cast<int64_t>(m.currentPos) + static_cast<int64_t>(distance);
  if (target <= INT32_MIN || target > INT32_MAX) return false;

  m.targetSpeedHz = speed;
  m.accelHz2 = accel;

  portENTER_CRITICAL(&timerMux);
  m.targetPos = m.currentPos + distance;
  m.moveDir = (distance > 0) ? 1 : -1;
  m.accumulator = 0;
  m.endstopHit = false;
  m.endCount = 0;
  m.watchEndstop = watch;
  portEXIT_CRITICAL(&timerMux);

  digitalWrite(m.dirPin, (m.moveDir > 0) ? m.positiveDirHigh : !m.positiveDirHigh);

  m.stopRequested = false;
  m.stopRetargeted = false;
  m.disableAfterStop = false;
  m.state = WAIT_ENABLE;
  enableMotor(m);
  return true;
}

void startMove(Motor &m, int32_t distance, uint32_t speed, uint32_t accel) {
  if (isBusy(m)) {
    Serial.print(F("error busy: ")); Serial.println(m.name);
    return;
  }
  const int64_t target = static_cast<int64_t>(m.currentPos) + static_cast<int64_t>(distance);
  if (target <= INT32_MIN || target > INT32_MAX) {
    Serial.println(F("error software position overflow; use zero while idle"));
    return;
  }

  if (distance == 0) {
    m.targetSpeedHz = speed;
    m.accelHz2 = accel;
    reportEnd(m, "done");
    return;
  }

  beginMove(m, distance, speed, accel, false);
  Serial.print(F("ok ")); Serial.print(m.name); Serial.println(F(" accepted"));
}

// ------------------------------------------------------------------
// HOMING
// ------------------------------------------------------------------
void abortHoming(Motor &m, const __FlashStringHelper *reason) {
  Serial.print(F("error homing failed ")); Serial.print(m.name);
  Serial.print(F(": ")); Serial.println(reason);
  m.homePhase = HOME_NONE;
  m.watchEndstop = false;
  // Cancel any remaining queued axes
  for (size_t i = 0; i < MOTOR_COUNT; ++i) motors[i].homeRequested = false;
  homeBatchActive = false;
}

// towardEndstop=true moves in homeDir, false moves away from the endstop
bool homeMove(Motor &m, bool towardEndstop, int32_t steps, uint32_t speed, bool watch) {
  int32_t dist = (towardEndstop ? m.homeDir : -m.homeDir) * steps;
  if (!beginMove(m, dist, speed, m.homeAccel, watch)) {
    abortHoming(m, F("could not start move"));
    return false;
  }
  return true;
}

void startHoming(Motor &m) {
  // Position is redefined by homing, so start from 0 to avoid overflow
  portENTER_CRITICAL(&timerMux);
  m.currentPos = 0;
  m.targetPos = 0;
  portEXIT_CRITICAL(&timerMux);

  Serial.print(F("homing ")); Serial.println(m.name);
  m.homeRetries = 0;
  if (endstopTriggered(m)) {
    // Already on the switch: retreat first
    m.homePhase = HOME_PRE_BACKOFF;
    homeMove(m, false, m.homeBackoff, m.homeSlowHz, false);
  } else {
    m.homePhase = HOME_FAST;
    homeMove(m, true, m.homeMaxTravel, m.homeFastHz, true);
  }
}

// Advances each homing axis when its current segment has finished
void serviceHoming() {
  for (size_t i = 0; i < MOTOR_COUNT; ++i) {
    Motor &m = motors[i];
    if (m.homePhase == HOME_NONE || m.state != IDLE) continue;

    switch (m.homePhase) {
      case HOME_PRE_BACKOFF:
        if (endstopTriggered(m)) {
          // Still on the switch: keep retreating (up to 6 times)
          if (++m.homeRetries >= 6) { abortHoming(m, F("endstop still triggered after retreat (check wiring/pull-up)")); break; }
          homeMove(m, false, m.homeBackoff, m.homeSlowHz, false);
          break;
        }
        m.homePhase = HOME_FAST;
        homeMove(m, true, m.homeMaxTravel, m.homeFastHz, true);
        break;

      case HOME_FAST:
        if (!m.endstopHit) { abortHoming(m, F("endstop not found within max travel")); break; }
        m.homePhase = HOME_BACKOFF;
        homeMove(m, false, m.homeBackoff, m.homeFastHz, false);
        break;

      case HOME_BACKOFF:
        if (endstopTriggered(m)) { abortHoming(m, F("endstop did not release on backoff")); break; }
        m.homePhase = HOME_SLOW;
        homeMove(m, true, m.homeBackoff * 2, m.homeSlowHz, true);
        break;

      case HOME_SLOW:
        if (!m.endstopHit) { abortHoming(m, F("endstop not found on slow approach")); break; }
        portENTER_CRITICAL(&timerMux);
        m.currentPos = 0;
        m.targetPos = 0;
        portEXIT_CRITICAL(&timerMux);
        m.homePhase = HOME_NONE;
        Serial.print(F("homed ")); Serial.print(m.name);
        Serial.println(F(" steps=0"));
        break;

      default: break;
    }
  }
}

// Starts the next requested axis (one at a time, in HOME_ORDER)
void serviceHomeQueue() {
  for (size_t i = 0; i < MOTOR_COUNT; ++i) {
    if (motors[i].homePhase != HOME_NONE) return;  // an axis is still homing
  }
  for (size_t k = 0; k < HOME_ORDER_COUNT; ++k) {
    Motor &m = motors[HOME_ORDER[k]];
    if (m.homeRequested && m.state == IDLE) {
      m.homeRequested = false;
      startHoming(m);
      return;
    }
  }
  if (homeBatchActive) {
    homeBatchActive = false;
    Serial.println(F("done g28"));
  }
}

void handleHome(char **axes, size_t n) {
  bool sel[MOTOR_COUNT] = { false };

  if (n == 0) {
    for (size_t i = 0; i < MOTOR_COUNT; ++i) sel[i] = (motors[i].endPin != NO_ENDSTOP);
  } else {
    for (size_t a = 0; a < n; ++a) {
      if (strcmp(axes[a], "all") == 0) {
        for (size_t i = 0; i < MOTOR_COUNT; ++i) sel[i] = (motors[i].endPin != NO_ENDSTOP);
        continue;
      }
      const int idx = findMotor(axes[a]);
      if (idx < 0) { Serial.println(F("error unknown motor ID")); return; }
      if (motors[idx].endPin == NO_ENDSTOP) {
        Serial.print(F("error no endstop on ")); Serial.println(motors[idx].name);
        return;
      }
      sel[idx] = true;
    }
  }

  for (size_t i = 0; i < MOTOR_COUNT; ++i) {
    if (sel[i] && isBusy(motors[i])) {
      Serial.print(F("error busy: ")); Serial.println(motors[i].name);
      return;
    }
  }

  Serial.print(F("ok g28 accepted:"));
  for (size_t k = 0; k < HOME_ORDER_COUNT; ++k) {
    const uint8_t idx = HOME_ORDER[k];
    if (sel[idx]) {
      motors[idx].homeRequested = true;
      Serial.print(' '); Serial.print(motors[idx].name);
    }
  }
  Serial.println();
  homeBatchActive = true;
}

void serviceMotors() {
  const uint32_t nowUs = micros();
  const uint32_t nowMs = millis();

  for (size_t i = 0; i < MOTOR_COUNT; ++i) {
    Motor &m = motors[i];

    if (m.state == WAIT_ENABLE && static_cast<uint32_t>(nowMs - m.enabledAt) >= ENABLE_SETTLE_MS) {
      m.currentSpeedHz = sqrtf(2.0f * static_cast<float>(m.accelHz2));
      if (m.currentSpeedHz > static_cast<float>(m.targetSpeedHz)) m.currentSpeedHz = m.targetSpeedHz;
      if (m.currentSpeedHz < 20.0f) m.currentSpeedHz = 20.0f;

      portENTER_CRITICAL(&timerMux);
      m.isrSpeed = static_cast<uint32_t>(m.currentSpeedHz);
      m.state = MOVING;
      portEXIT_CRITICAL(&timerMux);

      m.lastCalcUs = nowUs;
    }

    if (m.state == MOVING) {
      float dt = static_cast<float>(nowUs - m.lastCalcUs) / 1000000.0f;
      m.lastCalcUs = nowUs;

      portENTER_CRITICAL(&timerMux);
      int32_t cPos = m.currentPos;
      int32_t tPos = m.targetPos;
      portEXIT_CRITICAL(&timerMux);

      // STOP: pull the target in to the exact distance needed to decelerate
      // to rest at the requested accel, so the motor really stops.
      if (m.stopRequested && !m.stopRetargeted) {
        const float v = m.currentSpeedHz;
        const int32_t need = static_cast<int32_t>(ceilf((v * v) / (2.0f * static_cast<float>(m.accelHz2)))) + 2;
        portENTER_CRITICAL(&timerMux);
        cPos = m.currentPos;
        if (abs(m.targetPos - cPos) > need) m.targetPos = cPos + m.moveDir * need;
        tPos = m.targetPos;
        portEXIT_CRITICAL(&timerMux);
        m.stopRetargeted = true;
      }

      int32_t remaining = abs(tPos - cPos);

      if (remaining == 0) {
        portENTER_CRITICAL(&timerMux);
        m.state = IDLE;
        m.isrSpeed = 0;
        portEXIT_CRITICAL(&timerMux);

        m.currentSpeedHz = 0.0f;
        // Homing segments report through serviceHoming() instead
        if (m.homePhase == HOME_NONE) reportEnd(m, m.stopRequested ? "stopped" : "done");
        if (m.disableAfterStop) disableMotor(m);
      } else {
        float stoppingSteps = (m.currentSpeedHz * m.currentSpeedHz) / (2.0f * static_cast<float>(m.accelHz2));

        if (m.stopRequested || static_cast<float>(remaining) <= stoppingSteps + 1.0f) {
          m.currentSpeedHz -= static_cast<float>(m.accelHz2) * dt;
          float minSpeed = sqrtf(static_cast<float>(m.accelHz2));
          if (minSpeed < 20.0f) minSpeed = 20.0f;
          if (m.currentSpeedHz < minSpeed) m.currentSpeedHz = minSpeed;
        } else if (m.currentSpeedHz < static_cast<float>(m.targetSpeedHz)) {
          m.currentSpeedHz += static_cast<float>(m.accelHz2) * dt;
          if (m.currentSpeedHz > static_cast<float>(m.targetSpeedHz)) {
            m.currentSpeedHz = static_cast<float>(m.targetSpeedHz);
          }
        }

        portENTER_CRITICAL(&timerMux);
        m.isrSpeed = static_cast<uint32_t>(m.currentSpeedHz);
        portEXIT_CRITICAL(&timerMux);
      }
    }
  }
}

void printDiag() {
  static uint32_t lastTicks = 0, lastMs = 0;
  const uint32_t ticks = isrTicks;
  const uint32_t now = millis();
  const uint32_t ms = now - lastMs;
  const uint32_t rate = ms ? static_cast<uint32_t>((static_cast<uint64_t>(ticks - lastTicks) * 1000ULL) / ms) : 0;
  Serial.print(F("isr rate=")); Serial.print(rate);
  Serial.print(F(" Hz (target ")); Serial.print(ISR_FREQ);
  Serial.print(F(")  isr max cycles=")); Serial.print(isrMaxCycles);
  Serial.print(F(" (budget ")); Serial.print(cpuHz / ISR_FREQ);
  Serial.print(F(")  cpu=")); Serial.print(cpuHz / 1000000UL); Serial.println(F("MHz"));
  isrMaxCycles = 0;
  lastTicks = ticks;
  lastMs = now;
}

// ------------------------------------------------------------------
// NON-BLOCKING COORDINATE REPORT  ("pos" / "m114")
// The command only sets a flag. The reply is formatted into a buffer and
// handed to the UART in ONE write, and only when the TX buffer has room,
// so polling it at a high rate can never stall loop() or the motors.
// If the buffer is full, polls are coalesced into the next reply.
// Positions are read without disabling interrupts (aligned 32-bit reads).
// ------------------------------------------------------------------
bool posPending = false;
char posBuf[128];

void servicePosReply() {
  if (!posPending) return;
  if (Serial.availableForWrite() < static_cast<int>(sizeof(posBuf))) return;  // try again next loop

  size_t n = 0;
  n += snprintf(posBuf, sizeof(posBuf), "pos");
  for (size_t i = 0; i < MOTOR_COUNT && n < sizeof(posBuf); ++i) {
    const Motor &m = motors[i];
    const double units = static_cast<double>(m.currentPos) / m.stepsPerUnit;
    n += snprintf(posBuf + n, sizeof(posBuf) - n, " %c:%.*f",
                  static_cast<char>(toupper(static_cast<unsigned char>(m.name[0]))),
                  static_cast<int>(m.posDecimals), units);
  }
  if (n > sizeof(posBuf) - 2) n = sizeof(posBuf) - 2;
  posBuf[n++] = '\n';
  Serial.write(reinterpret_cast<const uint8_t *>(posBuf), n);
  posPending = false;
}

void printStatus(const Motor &m) {
  Serial.print(m.name);
  Serial.print(F(" EN=")); Serial.print(m.enabled ? F("on") : F("off"));
  Serial.print(F(" state="));
  switch (m.state) {
    case IDLE:        Serial.print(F("idle")); break;
    case WAIT_ENABLE: Serial.print(F("starting")); break;
    case MOVING:      Serial.print(m.stopRequested ? F("stopping") : F("moving")); break;
  }
  if (m.homePhase != HOME_NONE || m.homeRequested) Serial.print(F(" (homing)"));
  if (m.disableAfterStop) Serial.print(F(" (will disable)"));
  if (m.endPin != NO_ENDSTOP) {
    Serial.print(F(" endstop="));
    Serial.print(endstopTriggered(m) ? F("TRIGGERED") : F("open"));
  }
  Serial.print(F(" steps=")); Serial.print(m.currentPos);
  Serial.print(F(" pos=")); Serial.println(static_cast<double>(m.currentPos) / m.stepsPerUnit, m.posDecimals);
}

void handleLine(char *line) {
  char *comment = strchr(line, ';');
  if (comment) *comment = '\0';
  for (char *p = line; *p; ++p) *p = static_cast<char>(tolower(static_cast<unsigned char>(*p)));

  char *args[5];
  size_t count = 0;
  char *save = nullptr;
  for (char *word = strtok_r(line, " \t", &save); word;
       word = strtok_r(nullptr, " \t", &save)) {
    if (count == 5) { Serial.println(F("error too many arguments")); return; }
    args[count++] = word;
  }
  if (count == 0) return;

  if (strcmp(args[0], "help") == 0 && count == 1) { printHelp(); return; }
  if (strcmp(args[0], "diag") == 0 && count == 1) { printDiag(); return; }
  if ((strcmp(args[0], "pos") == 0 || strcmp(args[0], "m114") == 0) && count == 1) {
    posPending = true;   // reply is sent by servicePosReply() without blocking
    return;
  }

  if (strcmp(args[0], "g28") == 0) {
    handleHome(args + 1, count - 1);
    return;
  }

  if (strcmp(args[0], "g1") == 0 || strcmp(args[0], "g01") == 0) {
    if (count != 5) {
      Serial.println(F("error use: g1 <id> <distance> <speed> <accel>"));
      return;
    }
    const int index = findMotor(args[1]);
    if (index < 0) { Serial.println(F("error unknown motor ID")); return; }
    Motor &gm = motors[index];
    int32_t distance;
    uint32_t speed, accel;
    double dv, sv, av;
    if (!parseNumber(args[2], dv) || !parseNumber(args[3], sv) || !parseNumber(args[4], av) ||
        !unitsToSteps(gm, dv, distance) ||
        !unitsToRate(gm, sv, MAX_SPEED_HZ, speed) ||
        !unitsToRate(gm, av, MAX_ACCEL, accel)) {
      printG1Limits(gm);
      return;
    }
    startMove(motors[index], distance, speed, accel);
    return;
  }

  const bool en = strcmp(args[0], "enable") == 0;
  const bool stop = strcmp(args[0], "stop") == 0;
  const bool emergency = strcmp(args[0], "estop") == 0;
  const bool status = strcmp(args[0], "status") == 0;
  const bool zero = strcmp(args[0], "zero") == 0;
  if (!(en || stop || emergency || status || zero)) {
    Serial.println(F("error unknown command; type help")); return;
  }
  if ((en && count != 3) || ((stop || zero) && count != 2) ||
      ((emergency || status) && count != 1 && count != 2)) {
    Serial.println(F("error wrong argument count; type help")); return;
  }
  bool turnOn = false;
  if (en) {
    if (strcmp(args[2], "on") == 0) turnOn = true;
    else if (strcmp(args[2], "off") != 0) {
      Serial.println(F("error use: enable <id|all> on|off")); return;
    }
  }
  const char *target = count >= 2 ? args[1] : "all";
  const bool all = strcmp(target, "all") == 0;
  const int index = all ? -1 : findMotor(target);
  if (!all && index < 0) { Serial.println(F("error unknown motor ID")); return; }

  if (zero) {
    for (size_t i = 0; i < MOTOR_COUNT; ++i) {
      if ((all || static_cast<int>(i) == index) && isBusy(motors[i])) {
        Serial.println(F("error zero requires every selected motor to be idle"));
        return;
      }
    }
  }
  for (size_t i = 0; i < MOTOR_COUNT; ++i) {
    if (!all && static_cast<int>(i) != index) continue;
    Motor &m = motors[i];
    if (status) { printStatus(m); continue; }
    if (emergency) { emergencyStop(m); continue; }
    if (stop) requestStop(m, false);
    if (en) {
      if (turnOn) {
        enableMotor(m);
        m.disableAfterStop = false;
      } else requestStop(m, true);
    }
    if (zero) {
      portENTER_CRITICAL(&timerMux);
      m.currentPos = 0;
      portEXIT_CRITICAL(&timerMux);
    }
    Serial.print(F("ok ")); Serial.print(args[0]); Serial.print(' '); Serial.print(m.name);
    if (en) Serial.print(turnOn ? F(" on") : F(" off requested"));
    Serial.println();
  }
  if (emergency) {
    homeBatchActive = false;
    for (size_t i = 0; i < MOTOR_COUNT; ++i) {
      if (!all && static_cast<int>(i) != index) continue;
      Serial.print(F("aborted ")); Serial.print(motors[i].name);
      Serial.println(F("; physical position may be lost"));
    }
  }
}

void readSerial() {
  for (size_t n = 0; n < 64 && Serial.available(); ++n) {
    const char c = static_cast<char>(Serial.read());
    if (c == '!') {   // instant emergency stop, works even mid-line
      for (size_t i = 0; i < MOTOR_COUNT; ++i) emergencyStop(motors[i]);
      homeBatchActive = false;
      inputLength = 0;
      discardLine = false;
      Serial.println(F("ESTOP all (!); physical position may be lost"));
      continue;
    }
    if (c == '\r' || c == '\n') {
      if (discardLine) Serial.println(F("error line too long or invalid; discarded"));
      else if (inputLength) { inputLine[inputLength] = '\0'; handleLine(inputLine); }
      inputLength = 0;
      discardLine = false;
    } else if (!discardLine) {
      if ((static_cast<unsigned char>(c) < 32 && c != '\t') ||
          static_cast<unsigned char>(c) > 126 || inputLength >= LINE_SIZE - 1) {
        discardLine = true;
      } else inputLine[inputLength++] = c;
    }
  }
}

void setup() {
  for (size_t i = 0; i < MOTOR_COUNT; ++i) {
    pinMode(motors[i].enPin, OUTPUT);
    digitalWrite(motors[i].enPin, HIGH);

    pinMode(motors[i].stepPin, OUTPUT);
    digitalWrite(motors[i].stepPin, LOW);

    pinMode(motors[i].dirPin, OUTPUT);
    digitalWrite(motors[i].dirPin, motors[i].positiveDirHigh ? HIGH : LOW);

    if (motors[i].endPin != NO_ENDSTOP) {
      // GPIO34-39 are input-only with no internal pull resistors
      const bool inputOnly = (motors[i].endPin >= 34 && motors[i].endPin <= 39);
      if (inputOnly) {
        pinMode(motors[i].endPin, INPUT);
      } else {
        pinMode(motors[i].endPin, (ENDSTOP_TRIGGER_LEVEL == LOW) ? INPUT_PULLUP : INPUT_PULLDOWN);
      }
    }
  }
  Serial.setTxBufferSize(1024);   // must be before begin(); room for non-blocking replies
  Serial.begin(115200);

  cpuHz = static_cast<uint32_t>(getCpuFrequencyMhz()) * 1000000UL;
  maxDtCycles = cpuHz / 10000UL;   // clamp one ISR interval to 100 us

  // Initialize hardware timer (ESP32 Arduino Core v3.x API)
  stepTimer = timerBegin(1000000);
  timerAttachInterrupt(stepTimer, &onTimer);
  timerAlarm(stepTimer, ISR_PERIOD_US, true, 0);

  Serial.println(F("Six motor controller ready; type diag to check ISR rate."));
  printHelp();
}

void loop() {
  serviceMotors();
  serviceHoming();
  serviceHomeQueue();
  readSerial();
  servicePosReply();
}
