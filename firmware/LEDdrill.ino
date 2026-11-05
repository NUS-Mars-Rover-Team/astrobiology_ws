// --- Component Pins (MOSFETs) ---
const int uvm = 12;      // UVM Light
const int uvc = 11;      // UVC Light
const int laser = 10;    // Laser
const int halogen = 9;   // Halogen Light

// --- Motor/Drill Driver Pins ---
const int svPin  = 6;  // Connect to "SV" (Orange wire) for PWM Speed Control
const int frPin  = 8;  // Connect to "F/R" (White wire) for Direction
const int enPin  = 7;  // Connect to "EN" (Black wire) for Enable

// Set your desired running speed (0 to 255)
const int runSpeed = 150; 

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(50); // Speeds up string reading
  
  // --- Configure Component Pins ---
  pinMode(uvm, OUTPUT);
  pinMode(uvc, OUTPUT);
  pinMode(laser, OUTPUT);
  pinMode(halogen, OUTPUT);
  
  // Ensure everything is OFF initially
  digitalWrite(uvm, LOW);
  digitalWrite(uvc, LOW);
  digitalWrite(laser, LOW);
  digitalWrite(halogen, LOW);

  // --- Configure Motor Driver Pins ---
  pinMode(svPin, OUTPUT);
  pinMode(frPin, OUTPUT);
  pinMode(enPin, OUTPUT);
  
  // Start drill stopped and disabled
  digitalWrite(enPin, LOW); 
  analogWrite(svPin, 0);

  Serial.println("--- System Ready ---");
  Serial.println("Type commands to control the system:");
  Serial.println("Components: 'uvm on' / 'uvm off'");
  Serial.println("            'uvc on' / 'uvc off'");
  Serial.println("            'laser on' / 'laser off'");
  Serial.println("            'halogen on' / 'halogen off'");
  Serial.println("Drill:      'drill f' (Forward) | 'drill r' (Reverse) | 'drill s' (Stop)");
}

void loop() {
  if (Serial.available() > 0) {
    // Read the whole string until the user presses Enter
    String command = Serial.readStringUntil('\n');
    
    // Clean up the string (removes invisible carriage returns or extra spaces)
    command.trim(); 
    
    // Convert to lowercase so capitalization doesn't matter
    command.toLowerCase(); 
    
    // Ignore empty commands caused by serial monitor line endings
    if (command == "") return; 

    // ==================================
    //        COMPONENT COMMANDS
    // ==================================
    if (command == "uvm on") {
      digitalWrite(uvm, HIGH);
      Serial.println("UVM ON");
    } 
    else if (command == "uvm off") {
      digitalWrite(uvm, LOW);
      Serial.println("UVM OFF");
    }
    else if (command == "uvc on") {
      digitalWrite(uvc, HIGH);
      Serial.println("UVC ON");
    }
    else if (command == "uvc off") {
      digitalWrite(uvc, LOW);
      Serial.println("UVC OFF");
    }
    else if (command == "laser on") {
      digitalWrite(laser, HIGH);
      Serial.println("Laser ON");
    }
    else if (command == "laser off") {
      digitalWrite(laser, LOW);
      Serial.println("Laser OFF");
    }
    else if (command == "halogen on") {
      digitalWrite(halogen, HIGH);
      Serial.println("Halogen ON");
    }
    else if (command == "halogen off") {
      digitalWrite(halogen, LOW);
      Serial.println("Halogen OFF");
    }

    // ==================================
    //          DRILL COMMANDS
    // ==================================
    else if (command == "drill f") {
      digitalWrite(frPin, HIGH);
      digitalWrite(enPin, HIGH);
      analogWrite(svPin, runSpeed);
      Serial.println("Drill: FORWARD");
    }
    else if (command == "drill r") {
      digitalWrite(frPin, LOW);
      digitalWrite(enPin, HIGH);
      analogWrite(svPin, runSpeed);
      Serial.println("Drill: REVERSE");
    }
    else if (command == "drill s") {
      analogWrite(svPin, 0);
      digitalWrite(enPin, LOW);
      Serial.println("Drill: STOP");
    }
    
    // ==================================
    //          ERROR HANDLING
    // ==================================
    else {
      Serial.print("Invalid command: ");
      Serial.println(command);
    }
  }
}