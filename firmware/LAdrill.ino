// Pin mappings based on your driver board wiring
const int svPin  = 9;  // Connect to "SV" (Orange wire) for PWM Speed Control
const int frPin  = 8;  // Connect to "F/R" (White wire) for Direction
const int enPin  = 7;  // Connect to "EN" (Black wire) for Enable

// Set your desired running speed (0 to 255)
// Note: Increased to 200 just in case 150 was too low to start the motor
const int runSpeed = 200; 

void setup() {
  Serial.begin(9600);
  
  pinMode(svPin, OUTPUT);
  pinMode(frPin, OUTPUT);
  pinMode(enPin, OUTPUT);
  
  // Start stopped and disabled (FLIPPED LOGIC: HIGH = Disabled)
  digitalWrite(enPin, HIGH); 
  analogWrite(svPin, 0);
  
  Serial.println("Driver Ready! Send:");
  Serial.println(" 'f' = Forward");
  Serial.println(" 'b' = Backward");
  Serial.println(" 's' = Stop (Coast)");
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    // Ignore newline or carriage return characters
    if (command == '\n' || command == '\r') return; 

    if (command == 'f' || command == 'F') {
      digitalWrite(frPin, HIGH);         // Set Forward
      digitalWrite(enPin, LOW);          // Enable Driver (FLIPPED LOGIC: LOW = Enabled)
      analogWrite(svPin, runSpeed);      // Send PWM Speed
      Serial.println("Motor: FORWARD");
    } 
    else if (command == 'b' || command == 'B') {
      digitalWrite(frPin, LOW);          // Set Backward
      digitalWrite(enPin, LOW);          // Enable Driver (FLIPPED LOGIC: LOW = Enabled)
      analogWrite(svPin, runSpeed);      // Send PWM Speed
      Serial.println("Motor: BACKWARD");
    } 
    else if (command == 's' || command == 'S') {
      analogWrite(svPin, 0);             // Drop speed to 0
      digitalWrite(enPin, HIGH);         // Disable Driver (FLIPPED LOGIC: HIGH = Disabled)
      Serial.println("Motor: STOP (Coasting)");
    } 
    else {
      Serial.println("Invalid command. Use f, b, or s.");
    }
  }
}