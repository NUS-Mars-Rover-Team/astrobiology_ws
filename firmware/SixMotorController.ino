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

  Units (conversion ratios are hard-coded in the MOTOR TABLE below):
    x y z : mm, mm/s, mm/s^2     (200 steps/rev x 16 microstep / 1 mm pitch)
    c     : rev, RPM, RPM/s      (200 steps/rev x 8 microstep)
    p f   : pulses, pulses/s, pulses/s^2

  Coordinates:  pos  (or m114)  ->  pos Z:.. Y:.. X:.. C:.. P:.. F:..

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
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

const uint8_t NO_ENDSTOP = 0xFF;

// Homing settings, in RAW STEPS (not mm) - see HOMING below
struct HomeConfig {
  uint8_t endPin = NO_ENDSTOP;
  int8_t dir = -1;
  uint32_t fastHz = 0, slowHz = 0, accel = 0;
  int32_t backoff = 0, maxTravel = 0;
};

enum MotionState { IDLE, WAIT_ENABLE, MOVING };
enum HomePhase { HOME_NONE, HOME_PRE_BACKOFF, HOME_FAST, HOME_BACKOFF, HOME_SLOW };

struct Motor {
  // Configuration (MOTOR TABLE below)
  const char *name;
  const char *alias;
  uint8_t stepPin, dirPin, enPin;
  bool invertDir;
  double stepsPerUnit;
  double timeDiv;
  const char *unitLabel;
  HomeConfig home;

  // Kinematics (main loop only)
  bool enabled = false;
  bool disableAfterStop = false;
  uint32_t enabledAt = 0;
  uint32_t targetSpeedHz = 1000;
  uint32_t accelHz2 = 500;
  float currentSpeedHz = 0.0f;
  uint32_t lastCalcUs = 0;
  bool stopRequested = false;
  bool stopRetargeted = false;

  // Homing state (main loop only)
  HomePhase homePhase = HOME_NONE;
  bool homeRequested = false;
  uint8_t homeRetries = 0;

  // Real-time variables (Shared with ISR - must be volatile)
  volatile MotionState state = IDLE;
  volatile int32_t currentPos = 0;
  volatile int32_t targetPos = 0;
  volatile uint32_t isrSpeed = 0;
  volatile uint32_t accumulator = 0;
  volatile int8_t moveDir = 1;
  volatile bool stepPinHigh = false;
  volatile bool watchEndstop = false;
  volatile bool endstopHit = false;
  volatile uint8_t endCount = 0;

  bool hasEndstop() const { return home.endPin != NO_ENDSTOP; }
  bool dirLevel(int8_t dir) const { return (dir > 0) != invertDir; }   // DIR pin level for a step direction
  uint8_t decimals() const { return stepsPerUnit > 1.0 ? 4 : 0; }
  double units() const { return static_cast<double>(currentPos) / stepsPerUnit; }
};

// ==================================================================
//                        USER CONFIGURATION
// ==================================================================

// ---- ENDSTOPS ---------------------------------------------------
// Pin level when the endstop is PRESSED.
//   LOW  = switch connects pin to GND (pulled up otherwise)
//   HIGH = switch connects pin to 3.3V
const uint8_t ENDSTOP_TRIGGER_LEVEL = HIGH;

// Number of consecutive 4us ISR samples the pin must be triggered before it
// counts (simple debounce / noise filter). 8 = 32us.
const uint8_t ENDSTOP_DEBOUNCE_TICKS = 8;

// ---- HOMING (per axis) ------------------------------------------
// !! These are starting guesses - tune them for your machine !!
//   pin      : endstop GPIO
//   dir      : sign of the steps that move TOWARD the endstop
//   fast     : search speed (steps/s) until the endstop is found
//   slow     : final precise touch speed (steps/s)
//   accel    : acceleration used during homing (steps/s^2)
//   backoff  : steps to retreat after the first hit (must be enough to release)
//   maxTravel: give up if the endstop isn't found within this many steps
//                         pin  dir   fast  slow  accel  backoff  maxTravel
const HomeConfig HOME_Z = { 21, -1, 10000, 5000, 5000,   8000,   1000000 };
const HomeConfig HOME_Y = { 34, -1, 10000, 5000, 5000,   8000,   1000000 };
const HomeConfig HOME_X = { 35, -1, 10000, 5000, 5000,   8000,   1000000 };

// ---- MOTOR TABLE ------------------------------------------------
// invert   : false = positive steps drive DIR HIGH; true = DIR LOW (flips the motor)
// steps/unit = full steps per rev * microsteps (/ mm per rev for linear axes)
//   x y z : unit = mm   (200 steps/rev, 1/16 microstep, 1 mm pitch) = 3200 steps/mm
//   c     : unit = rev  (200 steps/rev, 1/8 microstep)              = 1600 steps/rev
//   p f   : unit = raw pulses (1:1)
// timeDiv  : time base of speed and accel: 1 = units/s (and units/s^2),
//            60 = units/min (and units/min per second), i.e. RPM and RPM/s.
// Motors sharing an EN pin are powered while any of them is enabled.
Motor motors[] = {
  // name alias      STEP DIR EN  invert  steps/unit  timeDiv unit      homing
  { "z", "z",           2,  4, 15, true,  200.0 * 16,  1,  "mm",     HOME_Z },
  { "y", "y",          16, 17, 15, false, 200.0 * 16,  1,  "mm",     HOME_Y },
  { "x", "x",          18, 19, 15, false, 200.0 * 16,  1,  "mm",     HOME_X },
  { "c", "centrifuge", 32, 33,  5, false, 200.0 * 8,  60,  "rev"    },
  { "p", "pump",       26, 27, 25, false, 1,           1,  "pulses" },
  { "f", "flush",      12, 13, 14, false, 1,           1,  "pulses" },
};

const size_t MOTOR_COUNT = sizeof(motors) / sizeof(motors[0]);

// Indices into motors[] (keep in sync with the table above)
enum { M_Z = 0, M_Y = 1, M_X = 2, M_C = 3, M_P = 4, M_F = 5 };

// Order in which axes are homed when several are requested (e.g. "g28").
// Z first (lift clear), then Y, then X. Edit to suit your machine.
const uint8_t HOME_ORDER[] = { M_Z, M_Y, M_X };

// ==================================================================

const uint32_t MAX_SPEED_HZ = 100000;
const uint32_t MAX_ACCEL = 1000000;
const float MIN_SPEED_HZ = 20.0f;
const uint32_t ENABLE_SETTLE_MS = 2;
const size_t LINE_SIZE = 160;

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

bool homeBatchActive = false;

char inputLine[LINE_SIZE];
size_t inputLength = 0;
bool discardLine = false;

// Masks the step ISR for the lifetime of the object; use around every
// main-loop access to the ISR-shared fields.
struct Critical {
  Critical() { portENTER_CRITICAL(&timerMux); }
  ~Critical() { portEXIT_CRITICAL(&timerMux); }
};

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
      if ((fastDigitalRead(m->home.endPin) ? HIGH : LOW) == ENDSTOP_TRIGGER_LEVEL) {
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

// ------------------------------------------------------------------
// HELPERS
// ------------------------------------------------------------------
bool eq(const char *a, const char *b) { return strcmp(a, b) == 0; }

int findMotor(const char *name) {
  for (size_t i = 0; i < MOTOR_COUNT; ++i) {
    if (eq(name, motors[i].name) || eq(name, motors[i].alias)) return static_cast<int>(i);
  }
  return -1;
}

bool isBusy(const Motor &m) {
  return m.state != IDLE || m.homePhase != HOME_NONE || m.homeRequested;
}

bool endstopTriggered(const Motor &m) {
  return digitalRead(m.home.endPin) == ENDSTOP_TRIGGER_LEVEL;
}

bool posOverflows(const Motor &m, int32_t distance) {
  const int64_t target = static_cast<int64_t>(m.currentPos) + distance;
  return target <= INT32_MIN || target > INT32_MAX;
}

void zeroPos(Motor &m) {
  Critical lock;
  m.currentPos = 0;
  m.targetPos = 0;
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

// ------------------------------------------------------------------
// SERIAL REPLIES
// ------------------------------------------------------------------
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

void printG1Limits(const Motor &m) {
  const bool perMin = (m.timeDiv > 1.0);
  Serial.print(F("error bad number for ")); Serial.print(m.name);
  Serial.print(F(": distance in ")); Serial.print(m.unitLabel);
  Serial.print(F(", speed 0..")); Serial.print(static_cast<double>(MAX_SPEED_HZ) * m.timeDiv / m.stepsPerUnit, 3);
  Serial.print(' '); Serial.print(m.unitLabel); Serial.print(perMin ? F("/min") : F("/s"));
  Serial.print(F(", accel 0..")); Serial.print(static_cast<double>(MAX_ACCEL) * m.timeDiv / m.stepsPerUnit, 3);
  Serial.print(' '); Serial.print(m.unitLabel); Serial.println(perMin ? F("/min/s") : F("/s^2"));
}

void printBusy(const Motor &m) {
  Serial.print(F("error busy: ")); Serial.println(m.name);
}

// " steps=<n> pos=<units>" and end the line
void printStepsPos(const Motor &m) {
  Serial.print(F(" steps=")); Serial.print(m.currentPos);
  Serial.print(F(" pos=")); Serial.println(m.units(), m.decimals());
}

void reportEnd(const Motor &m, const char *event) {
  Serial.print(event); Serial.print(' '); Serial.print(m.name);
  printStepsPos(m);
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
  if (m.hasEndstop()) {
    Serial.print(F(" endstop="));
    Serial.print(endstopTriggered(m) ? F("TRIGGERED") : F("open"));
  }
  printStepsPos(m);
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

  size_t n = snprintf(posBuf, sizeof(posBuf), "pos");
  for (size_t i = 0; i < MOTOR_COUNT && n < sizeof(posBuf); ++i) {
    const Motor &m = motors[i];
    n += snprintf(posBuf + n, sizeof(posBuf) - n, " %c:%.*f",
                  static_cast<char>(toupper(static_cast<unsigned char>(m.name[0]))),
                  static_cast<int>(m.decimals()), m.units());
  }
  if (n > sizeof(posBuf) - 2) n = sizeof(posBuf) - 2;
  posBuf[n++] = '\n';
  Serial.write(reinterpret_cast<const uint8_t *>(posBuf), n);
  posPending = false;
}

// ------------------------------------------------------------------
// ENABLE / STOP / MOVE
// ------------------------------------------------------------------
void syncHardwareEnablePins() {
  for (const Motor &m : motors) {
    bool anyEnabledOnThisPin = false;
    for (const Motor &other : motors) anyEnabledOnThisPin |= (other.enPin == m.enPin && other.enabled);
    digitalWrite(m.enPin, anyEnabledOnThisPin ? LOW : HIGH);
  }
}

void enableMotor(Motor &m) {
  if (m.enabled) return;
  m.enabled = true;
  m.enabledAt = millis();
  syncHardwareEnablePins();
}

void disableMotor(Motor &m) {
  m.enabled = false;
  m.disableAfterStop = false;
  syncHardwareEnablePins();
}

void requestStop(Motor &m, bool release) {
  // Any stop cancels homing on this axis (and its pending queue entry)
  m.homeRequested = false;
  m.homePhase = HOME_NONE;
  m.watchEndstop = false;
  m.disableAfterStop |= release;

  if (m.state == MOVING) { m.stopRequested = true; return; }
  if (m.state == WAIT_ENABLE) {
    m.state = IDLE;
    reportEnd(m, "cancelled");
  }
  if (m.disableAfterStop) disableMotor(m);
}

void emergencyStop(Motor &m) {
  {
    Critical lock;
    m.state = IDLE;
    m.isrSpeed = 0;
    m.watchEndstop = false;
    m.targetPos = m.currentPos;
    fastDigitalWrite(m.stepPin, false);
    m.stepPinHigh = false;
  }
  m.homePhase = HOME_NONE;
  m.homeRequested = false;
  m.currentSpeedHz = 0.0f;
  m.disableAfterStop = false;
  m.enabled = false;
  syncHardwareEnablePins();
}

// Low-level move start (no serial output). Returns false if it can't start.
bool beginMove(Motor &m, int32_t distance, uint32_t speed, uint32_t accel, bool watch) {
  if (m.state != IDLE || distance == 0 || posOverflows(m, distance)) return false;

  m.targetSpeedHz = speed;
  m.accelHz2 = accel;
  {
    Critical lock;
    m.targetPos = m.currentPos + distance;
    m.moveDir = (distance > 0) ? 1 : -1;
    m.accumulator = 0;
    m.endstopHit = false;
    m.endCount = 0;
    m.watchEndstop = watch;
  }
  digitalWrite(m.dirPin, m.dirLevel(m.moveDir));

  m.stopRequested = false;
  m.stopRetargeted = false;
  m.disableAfterStop = false;
  m.state = WAIT_ENABLE;
  enableMotor(m);
  return true;
}

void startMove(Motor &m, int32_t distance, uint32_t speed, uint32_t accel) {
  if (isBusy(m)) { printBusy(m); return; }
  if (posOverflows(m, distance)) {
    Serial.println(F("error software position overflow; use zero while idle"));
    return;
  }
  if (distance == 0) { reportEnd(m, "done"); return; }

  beginMove(m, distance, speed, accel, false);
  Serial.print(F("ok ")); Serial.print(m.name); Serial.println(F(" accepted"));
}

// Accelerates, cruises and decelerates every moving motor; called from loop()
void serviceMotors() {
  const uint32_t nowUs = micros();
  const uint32_t nowMs = millis();

  for (Motor &m : motors) {
    const float accel = static_cast<float>(m.accelHz2);
    const float target = static_cast<float>(m.targetSpeedHz);

    if (m.state == WAIT_ENABLE && nowMs - m.enabledAt >= ENABLE_SETTLE_MS) {
      m.currentSpeedHz = fmaxf(fminf(sqrtf(2.0f * accel), target), MIN_SPEED_HZ);
      {
        Critical lock;
        m.isrSpeed = static_cast<uint32_t>(m.currentSpeedHz);
        m.state = MOVING;
      }
      m.lastCalcUs = nowUs;
    }
    if (m.state != MOVING) continue;

    const float dt = static_cast<float>(nowUs - m.lastCalcUs) / 1000000.0f;
    m.lastCalcUs = nowUs;

    // STOP: pull the target in to the exact distance needed to decelerate
    // to rest at the requested accel, so the motor really stops.
    int32_t stopSteps = -1;
    if (m.stopRequested && !m.stopRetargeted) {
      stopSteps = static_cast<int32_t>(ceilf(m.currentSpeedHz * m.currentSpeedHz / (2.0f * accel))) + 2;
      m.stopRetargeted = true;
    }
    int32_t cPos, tPos;
    {
      Critical lock;
      cPos = m.currentPos;
      if (stopSteps >= 0 && abs(m.targetPos - cPos) > stopSteps) m.targetPos = cPos + m.moveDir * stopSteps;
      tPos = m.targetPos;
    }

    const int32_t remaining = abs(tPos - cPos);
    if (remaining == 0) {
      {
        Critical lock;
        m.state = IDLE;
        m.isrSpeed = 0;
      }
      m.currentSpeedHz = 0.0f;
      // Homing segments report through serviceHoming() instead
      if (m.homePhase == HOME_NONE) reportEnd(m, m.stopRequested ? "stopped" : "done");
      if (m.disableAfterStop) disableMotor(m);
      continue;
    }

    const float stoppingSteps = m.currentSpeedHz * m.currentSpeedHz / (2.0f * accel);
    if (m.stopRequested || static_cast<float>(remaining) <= stoppingSteps + 1.0f) {
      m.currentSpeedHz = fmaxf(m.currentSpeedHz - accel * dt, fmaxf(sqrtf(accel), MIN_SPEED_HZ));
    } else if (m.currentSpeedHz < target) {
      m.currentSpeedHz = fminf(m.currentSpeedHz + accel * dt, target);
    }
    {
      Critical lock;
      m.isrSpeed = static_cast<uint32_t>(m.currentSpeedHz);
    }
  }
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
  for (Motor &other : motors) other.homeRequested = false;
  homeBatchActive = false;
}

// towardEndstop=true moves in home.dir, false moves away from the endstop
bool homeMove(Motor &m, bool towardEndstop, int32_t steps, uint32_t speed, bool watch) {
  const int32_t dist = (towardEndstop ? m.home.dir : -m.home.dir) * steps;
  if (!beginMove(m, dist, speed, m.home.accel, watch)) {
    abortHoming(m, F("could not start move"));
    return false;
  }
  return true;
}

void retreat(Motor &m) { homeMove(m, false, m.home.backoff, m.home.slowHz, false); }

void fastSearch(Motor &m) {
  m.homePhase = HOME_FAST;
  homeMove(m, true, m.home.maxTravel, m.home.fastHz, true);
}

void startHoming(Motor &m) {
  zeroPos(m);   // position is redefined by homing, so start from 0 to avoid overflow
  Serial.print(F("homing ")); Serial.println(m.name);
  m.homeRetries = 0;
  if (endstopTriggered(m)) {
    // Already on the switch: retreat first
    m.homePhase = HOME_PRE_BACKOFF;
    retreat(m);
  } else {
    fastSearch(m);
  }
}

// Advances each homing axis when its current segment has finished
void serviceHoming() {
  for (Motor &m : motors) {
    if (m.homePhase == HOME_NONE || m.state != IDLE) continue;

    switch (m.homePhase) {
      case HOME_PRE_BACKOFF:
        if (!endstopTriggered(m)) fastSearch(m);
        // Still on the switch: keep retreating (up to 6 times)
        else if (++m.homeRetries >= 6) abortHoming(m, F("endstop still triggered after retreat (check wiring/pull-up)"));
        else retreat(m);
        break;

      case HOME_FAST:
        if (!m.endstopHit) { abortHoming(m, F("endstop not found within max travel")); break; }
        m.homePhase = HOME_BACKOFF;
        homeMove(m, false, m.home.backoff, m.home.fastHz, false);
        break;

      case HOME_BACKOFF:
        if (endstopTriggered(m)) { abortHoming(m, F("endstop did not release on backoff")); break; }
        m.homePhase = HOME_SLOW;
        homeMove(m, true, m.home.backoff * 2, m.home.slowHz, true);
        break;

      case HOME_SLOW:
        if (!m.endstopHit) { abortHoming(m, F("endstop not found on slow approach")); break; }
        zeroPos(m);
        m.homePhase = HOME_NONE;
        Serial.print(F("homed ")); Serial.print(m.name); Serial.println(F(" steps=0"));
        break;

      default: break;
    }
  }
}

// Starts the next requested axis (one at a time, in HOME_ORDER)
void serviceHomeQueue() {
  for (const Motor &m : motors) {
    if (m.homePhase != HOME_NONE) return;  // an axis is still homing
  }
  for (uint8_t idx : HOME_ORDER) {
    Motor &m = motors[idx];
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

// ------------------------------------------------------------------
// COMMANDS
// ------------------------------------------------------------------
void handleHome(char **axes, size_t n) {
  bool sel[MOTOR_COUNT] = { false };
  bool all = (n == 0);
  for (size_t a = 0; a < n; ++a) {
    if (eq(axes[a], "all")) { all = true; continue; }
    const int idx = findMotor(axes[a]);
    if (idx < 0) { Serial.println(F("error unknown motor ID")); return; }
    if (!motors[idx].hasEndstop()) {
      Serial.print(F("error no endstop on ")); Serial.println(motors[idx].name);
      return;
    }
    sel[idx] = true;
  }
  for (size_t i = 0; i < MOTOR_COUNT; ++i) {
    if (all && motors[i].hasEndstop()) sel[i] = true;
    if (sel[i] && isBusy(motors[i])) { printBusy(motors[i]); return; }
  }

  Serial.print(F("ok g28 accepted:"));
  for (uint8_t idx : HOME_ORDER) {
    if (!sel[idx]) continue;
    motors[idx].homeRequested = true;
    Serial.print(' '); Serial.print(motors[idx].name);
  }
  Serial.println();
  homeBatchActive = true;
}

void handleG1(char **args, size_t count) {
  if (count != 5) {
    Serial.println(F("error use: g1 <id> <distance> <speed> <accel>"));
    return;
  }
  const int index = findMotor(args[1]);
  if (index < 0) { Serial.println(F("error unknown motor ID")); return; }
  Motor &m = motors[index];
  int32_t distance;
  uint32_t speed, accel;
  double dv, sv, av;
  if (!parseNumber(args[2], dv) || !parseNumber(args[3], sv) || !parseNumber(args[4], av) ||
      !unitsToSteps(m, dv, distance) ||
      !unitsToRate(m, sv, MAX_SPEED_HZ, speed) ||
      !unitsToRate(m, av, MAX_ACCEL, accel)) {
    printG1Limits(m);
    return;
  }
  startMove(m, distance, speed, accel);
}

void handleLine(char *line) {
  char *comment = strchr(line, ';');
  if (comment) *comment = '\0';
  for (char *p = line; *p; ++p) *p = static_cast<char>(tolower(static_cast<unsigned char>(*p)));

  char *args[5];
  size_t count = 0;
  char *save = nullptr;
  for (char *word = strtok_r(line, " \t", &save); word; word = strtok_r(nullptr, " \t", &save)) {
    if (count == 5) { Serial.println(F("error too many arguments")); return; }
    args[count++] = word;
  }
  if (count == 0) return;
  const char *cmd = args[0];

  if (count == 1 && eq(cmd, "help")) { printHelp(); return; }
  if (count == 1 && eq(cmd, "diag")) { printDiag(); return; }
  if (count == 1 && (eq(cmd, "pos") || eq(cmd, "m114"))) {
    posPending = true;   // reply is sent by servicePosReply() without blocking
    return;
  }
  if (eq(cmd, "g28")) { handleHome(args + 1, count - 1); return; }
  if (eq(cmd, "g1") || eq(cmd, "g01")) { handleG1(args, count); return; }

  // enable / stop / estop / status / zero  <id|all>
  const bool en = eq(cmd, "enable");
  const bool stop = eq(cmd, "stop");
  const bool emergency = eq(cmd, "estop");
  const bool status = eq(cmd, "status");
  const bool zero = eq(cmd, "zero");
  if (!(en || stop || emergency || status || zero)) {
    Serial.println(F("error unknown command; type help")); return;
  }
  if ((en && count != 3) || ((stop || zero) && count != 2) || ((emergency || status) && count > 2)) {
    Serial.println(F("error wrong argument count; type help")); return;
  }
  const bool turnOn = en && eq(args[2], "on");
  if (en && !turnOn && !eq(args[2], "off")) {
    Serial.println(F("error use: enable <id|all> on|off")); return;
  }
  const bool all = count < 2 || eq(args[1], "all");
  const int index = all ? -1 : findMotor(args[1]);
  if (!all && index < 0) { Serial.println(F("error unknown motor ID")); return; }
  auto selected = [&](size_t i) { return all || static_cast<int>(i) == index; };

  if (zero) {
    for (size_t i = 0; i < MOTOR_COUNT; ++i) {
      if (selected(i) && isBusy(motors[i])) {
        Serial.println(F("error zero requires every selected motor to be idle"));
        return;
      }
    }
  }
  for (size_t i = 0; i < MOTOR_COUNT; ++i) {
    if (!selected(i)) continue;
    Motor &m = motors[i];
    if (status) { printStatus(m); continue; }
    if (emergency) { emergencyStop(m); continue; }
    if (stop) requestStop(m, false);
    if (en && turnOn) { enableMotor(m); m.disableAfterStop = false; }
    if (en && !turnOn) requestStop(m, true);
    if (zero) zeroPos(m);
    Serial.print(F("ok ")); Serial.print(cmd); Serial.print(' '); Serial.print(m.name);
    if (en) Serial.print(turnOn ? F(" on") : F(" off requested"));
    Serial.println();
  }
  if (emergency) {
    // Reported after every selected motor has stopped
    homeBatchActive = false;
    for (size_t i = 0; i < MOTOR_COUNT; ++i) {
      if (!selected(i)) continue;
      Serial.print(F("aborted ")); Serial.print(motors[i].name);
      Serial.println(F("; physical position may be lost"));
    }
  }
}

void readSerial() {
  for (size_t n = 0; n < 64 && Serial.available(); ++n) {
    const char c = static_cast<char>(Serial.read());
    if (c == '!') {   // instant emergency stop, works even mid-line
      for (Motor &m : motors) emergencyStop(m);
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
  for (const Motor &m : motors) {
    pinMode(m.enPin, OUTPUT);   digitalWrite(m.enPin, HIGH);   // drivers off
    pinMode(m.stepPin, OUTPUT); digitalWrite(m.stepPin, LOW);
    pinMode(m.dirPin, OUTPUT);  digitalWrite(m.dirPin, m.dirLevel(1));
    if (m.hasEndstop()) {
      // GPIO34-39 are input-only with no internal pull resistors
      const uint8_t pin = m.home.endPin;
      pinMode(pin, pin >= 34 ? INPUT : (ENDSTOP_TRIGGER_LEVEL == LOW) ? INPUT_PULLUP : INPUT_PULLDOWN);
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
