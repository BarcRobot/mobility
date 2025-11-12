/*
 * Linear Actuator Control System
 * 
 * Controls PA-MC1 linear actuator via Cytron MD10C motor driver
 * Receives commands from Raspberry Pi via Serial
 * 
 * Hardware Connections:
 * Motor Driver (MD10C) -> Arduino:
 *   - PWM pin -> D5 (PWM control)
 *   - DIR pin -> D4 (Direction control)
 *   - GND -> GND
 * 
 * Motor Driver -> Actuator:
 *   - Motor output terminals to actuator
 * 
 * Motor Driver -> Power:
 *   - Connect 12V/24V power supply (based on actuator voltage)
 * 
 * Communication Protocol:
 *   Send commands via Serial (9600 baud):
 *   - "EXTEND" - Extend actuator
 *   - "RETRACT" - Retract actuator
 *   - "STOP" - Stop movement
 *   - "SPEED:xxx" - Set speed (0-255)
 *   - "EXTEND:xxxx" - Extend for x milliseconds
 *   - "RETRACT:xxxx" - Retract for x milliseconds
 */

// Motor driver pin definitions
const int PWM_PIN = 5;   // PWM speed control
const int DIR_PIN = 4;   // Direction control

// Speed settings
int motorSpeed = 200;    // Default speed (0-255)
const int MIN_SPEED = 50;
const int MAX_SPEED = 255;

// Direction definitions
const bool EXTEND = HIGH;
const bool RETRACT = LOW;

// Timing variables
unsigned long moveStartTime = 0;
unsigned long moveDuration = 0;
bool timedMove = false;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Configure motor driver pins
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  
  // Start with motor stopped
  stopMotor();
  
  Serial.println("Linear Actuator Control Ready");
  Serial.println("Commands: EXTEND, RETRACT, STOP, SPEED:xxx");
  Serial.println("Timed commands: EXTEND:xxxx, RETRACT:xxxx (time in ms)");
}

void loop() {
  // Check for timed move completion
  if (timedMove && (millis() - moveStartTime >= moveDuration)) {
    stopMotor();
    timedMove = false;
    Serial.println("Timed move complete");
  }
  
  // Check for incoming serial commands
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    processCommand(command);
  }
}

void processCommand(String cmd) {
  cmd.toUpperCase();
  
  if (cmd == "EXTEND") {
    extendActuator();
    Serial.println("Extending actuator");
  }
  else if (cmd == "RETRACT") {
    retractActuator();
    Serial.println("Retracting actuator");
  }
  else if (cmd == "STOP") {
    stopMotor();
    Serial.println("Motor stopped");
  }
  else if (cmd.startsWith("SPEED:")) {
    int newSpeed = cmd.substring(6).toInt();
    setSpeed(newSpeed);
  }
  else if (cmd.startsWith("EXTEND:")) {
    int duration = cmd.substring(7).toInt();
    timedExtend(duration);
  }
  else if (cmd.startsWith("RETRACT:")) {
    int duration = cmd.substring(8).toInt();
    timedRetract(duration);
  }
  else {
    Serial.println("Unknown command: " + cmd);
  }
}

void extendActuator() {
  timedMove = false;
  digitalWrite(DIR_PIN, EXTEND);
  analogWrite(PWM_PIN, motorSpeed);
}

void retractActuator() {
  timedMove = false;
  digitalWrite(DIR_PIN, RETRACT);
  analogWrite(PWM_PIN, motorSpeed);
}

void stopMotor() {
  analogWrite(PWM_PIN, 0);
  timedMove = false;
}

void setSpeed(int speed) {
  if (speed < MIN_SPEED) speed = MIN_SPEED;
  if (speed > MAX_SPEED) speed = MAX_SPEED;
  
  motorSpeed = speed;
  Serial.print("Speed set to: ");
  Serial.println(motorSpeed);
}

void timedExtend(int duration) {
  if (duration > 0) {
    moveStartTime = millis();
    moveDuration = duration;
    timedMove = true;
    extendActuator();
    Serial.print("Extending for ");
    Serial.print(duration);
    Serial.println(" ms");
  }
}

void timedRetract(int duration) {
  if (duration > 0) {
    moveStartTime = millis();
    moveDuration = duration;
    timedMove = true;
    retractActuator();
    Serial.print("Retracting for ");
    Serial.print(duration);
    Serial.println(" ms");
  }
}