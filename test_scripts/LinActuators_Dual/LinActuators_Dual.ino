/*
 * Dual Linear Actuator Control System
 * 
 * Controls 2x PA-MC1 linear actuators via Cytron MD10C motor driver
 * Receives commands from Raspberry Pi via Serial
 * 
 * Hardware Connections:
 * Motor Driver (MD10C) -> Arduino:
 *   Actuator 1:
 *     - PWM pin -> D5 (PWM control)
 *     - DIR pin -> D4 (Direction control)
 *   Actuator 2:
 *     - PWM pin -> D6 (PWM control)
 *     - DIR pin -> D7 (Direction control)
 *   - GND -> GND
 * 
 * Motor Driver -> Actuators:
 *   - Channel 1 motor outputs to Actuator 1
 *   - Channel 2 motor outputs to Actuator 2
 * 
 * Motor Driver -> Power:
 *   - Connect 12V/24V power supply (based on actuator voltage)
 * 
 * Communication Protocol:
 *   Send commands via Serial (9600 baud):
 *   - "1:EXTEND" - Extend actuator 1
 *   - "2:RETRACT" - Retract actuator 2
 *   - "BOTH:EXTEND" - Extend both actuators
 *   - "BOTH:RETRACT" - Retract both actuators
 *   - "1:STOP" - Stop actuator 1
 *   - "BOTH:STOP" - Stop both actuators
 *   - "1:SPEED:xxx" - Set speed for actuator 1 (0-255)
 *   - "BOTH:SPEED:xxx" - Set speed for both actuators
 *   - "1:EXTEND:xxxx" - Extend actuator 1 for x milliseconds
 *   - "BOTH:EXTEND:xxxx" - Extend both for x milliseconds
 */

// Motor driver pin definitions
// Actuator 1
const int PWM_PIN_1 = 5;
const int DIR_PIN_1 = 4;

// Actuator 2
const int PWM_PIN_2 = 7;
const int DIR_PIN_2 = 6;

// Actuator structure
struct Actuator {
  int pwmPin;
  int dirPin;
  int speed;
  unsigned long moveStartTime;
  unsigned long moveDuration;
  bool timedMove;
};

// Initialize actuators
Actuator actuator1 = {PWM_PIN_1, DIR_PIN_1, 200, 0, 0, false};
Actuator actuator2 = {PWM_PIN_2, DIR_PIN_2, 200, 0, 0, false};

// Speed settings
const int MIN_SPEED = 50;
const int MAX_SPEED = 255;

// Direction definitions
const bool EXTEND = HIGH;
const bool RETRACT = LOW;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Configure motor driver pins for actuator 1
  pinMode(actuator1.pwmPin, OUTPUT);
  pinMode(actuator1.dirPin, OUTPUT);
  
  // Configure motor driver pins for actuator 2
  pinMode(actuator2.pwmPin, OUTPUT);
  pinMode(actuator2.dirPin, OUTPUT);
  
  // Start with motors stopped
  stopMotor(&actuator1);
  stopMotor(&actuator2);
  
  Serial.println("Dual Linear Actuator Control Ready");
  Serial.println("Commands: 1:EXTEND, 2:RETRACT, BOTH:STOP, etc.");
  Serial.println("Speed: 1:SPEED:xxx, BOTH:SPEED:xxx");
  Serial.println("Timed: 1:EXTEND:xxxx, BOTH:RETRACT:xxxx (time in ms)");
}

void loop() {
  // Check for timed move completion on actuator 1
  checkTimedMove(&actuator1, 1);
  
  // Check for timed move completion on actuator 2
  checkTimedMove(&actuator2, 2);
  
  // Check for incoming serial commands
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    processCommand(command);
  }
}

void checkTimedMove(Actuator* act, int actNum) {
  if (act->timedMove && (millis() - act->moveStartTime >= act->moveDuration)) {
    stopMotor(act);
    act->timedMove = false;
    Serial.print("Actuator ");
    Serial.print(actNum);
    Serial.println(" timed move complete");
  }
}

void processCommand(String cmd) {
  cmd.toUpperCase();
  
  // Parse actuator number and command
  int colonPos = cmd.indexOf(':');
  if (colonPos == -1) {
    Serial.println("Invalid command format. Use: 1:COMMAND or BOTH:COMMAND");
    return;
  }
  
  String target = cmd.substring(0, colonPos);
  String action = cmd.substring(colonPos + 1);
  
  // Determine which actuator(s) to control
  bool control1 = (target == "1" || target == "BOTH");
  bool control2 = (target == "2" || target == "BOTH");
  
  if (!control1 && !control2) {
    Serial.println("Invalid target. Use: 1, 2, or BOTH");
    return;
  }
  
  // Process the action
  if (action == "EXTEND") {
    if (control1) extendActuator(&actuator1, 1);
    if (control2) extendActuator(&actuator2, 2);
  }
  else if (action == "RETRACT") {
    if (control1) retractActuator(&actuator1, 1);
    if (control2) retractActuator(&actuator2, 2);
  }
  else if (action == "STOP") {
    if (control1) {
      stopMotor(&actuator1);
      Serial.println("Actuator 1 stopped");
    }
    if (control2) {
      stopMotor(&actuator2);
      Serial.println("Actuator 2 stopped");
    }
  }
  else if (action.startsWith("SPEED:")) {
    int newSpeed = action.substring(6).toInt();
    if (control1) setSpeed(&actuator1, newSpeed, 1);
    if (control2) setSpeed(&actuator2, newSpeed, 2);
  }
  else if (action.startsWith("EXTEND:")) {
    int duration = action.substring(7).toInt();
    if (control1) timedExtend(&actuator1, duration, 1);
    if (control2) timedExtend(&actuator2, duration, 2);
  }
  else if (action.startsWith("RETRACT:")) {
    int duration = action.substring(8).toInt();
    if (control1) timedRetract(&actuator1, duration, 1);
    if (control2) timedRetract(&actuator2, duration, 2);
  }
  else {
    Serial.println("Unknown action: " + action);
  }
}

void extendActuator(Actuator* act, int actNum) {
  act->timedMove = false;
  digitalWrite(act->dirPin, EXTEND);
  analogWrite(act->pwmPin, act->speed);
  Serial.print("Actuator ");
  Serial.print(actNum);
  Serial.println(" extending");
}

void retractActuator(Actuator* act, int actNum) {
  act->timedMove = false;
  digitalWrite(act->dirPin, RETRACT);
  analogWrite(act->pwmPin, act->speed);
  Serial.print("Actuator ");
  Serial.print(actNum);
  Serial.println(" retracting");
}

void stopMotor(Actuator* act) {
  analogWrite(act->pwmPin, 0);
  act->timedMove = false;
}

void setSpeed(Actuator* act, int speed, int actNum) {
  if (speed < MIN_SPEED) speed = MIN_SPEED;
  if (speed > MAX_SPEED) speed = MAX_SPEED;
  
  act->speed = speed;
  Serial.print("Actuator ");
  Serial.print(actNum);
  Serial.print(" speed set to: ");
  Serial.println(speed);
}

void timedExtend(Actuator* act, int duration, int actNum) {
  if (duration > 0) {
    act->moveStartTime = millis();
    act->moveDuration = duration;
    act->timedMove = true;
    extendActuator(act, actNum);
    Serial.print("Actuator ");
    Serial.print(actNum);
    Serial.print(" extending for ");
    Serial.print(duration);
    Serial.println(" ms");
  }
}

void timedRetract(Actuator* act, int duration, int actNum) {
  if (duration > 0) {
    act->moveStartTime = millis();
    act->moveDuration = duration;
    act->timedMove = true;
    retractActuator(act, actNum);
    Serial.print("Actuator ");
    Serial.print(actNum);
    Serial.print(" retracting for ");
    Serial.print(duration);
    Serial.println(" ms");
  }
}