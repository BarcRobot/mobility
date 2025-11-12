/*
 * Combined Actuator and Cleaning Motor Control System
 * 
 * Controls 2x PA-MC1 linear actuators + 2x cleaning brush motors
 * Receives state commands from Raspberry Pi via Serial
 * 
 * Hardware Connections:
 * 
 * LINEAR ACTUATORS (via MD10C Motor Driver):
 *   Actuator 1: PWM -> D5, DIR -> D4
 *   Actuator 2: PWM -> D6, DIR -> D7
 * 
 * CLEANING MOTORS (via separate motor driver):
 *   Motor A (Brush A): PWM -> D9, DIR -> D2
 *   Motor B (Brush B): PWM -> D10, DIR -> D3
 * 
 * SYSTEM STATES:
 *   IDLE          - All motors stopped
 *   EXTEND        - Extend both actuators
 *   RETRACT       - Retract both actuators
 *   CLEAN         - Run cleaning motors
 * 
 * Commands from Raspberry Pi (115200 baud):
 *   STATE,IDLE           - Stop everything
 *   STATE,EXTEND         - Extend actuators
 *   STATE,RETRACT        - Retract actuators
 *   STATE,CLEAN          - Run cleaning motors only
 *   
 *   ACT_SPEED,xxx        - Set actuator speed (50-255)
 *   CLEAN_SPEED,pwmA,pwmB - Set cleaning motor speeds (-255 to 255)
 *   
 *   P                    - Ping (returns PONG)
 *   I                    - Get system info
 *   S                    - Emergency stop all
 */

// ===== PIN DEFINITIONS =====
// Linear Actuators
const int ACT1_PWM = 5;
const int ACT1_DIR = 4;
const int ACT2_PWM = 7;
const int ACT2_DIR = 6;

// Cleaning Motors
const int BRUSH_A_PWM = 9;
const int BRUSH_A_DIR = 2;
const int BRUSH_B_PWM = 10;
const int BRUSH_B_DIR = 3;

// ===== SYSTEM STATES =====
enum SystemState {
  IDLE,
  EXTEND,
  RETRACT,
  CLEAN
};

SystemState currentState = IDLE;
String stateNames[] = {"IDLE", "EXTEND", "RETRACT", "CLEAN"};

// ===== ACTUATOR SETTINGS =====
int actuatorSpeed = 200;
const int MIN_ACT_SPEED = 50;
const int MAX_ACT_SPEED = 255;
const bool ACT_EXTEND = HIGH;
const bool ACT_RETRACT = LOW;

// ===== CLEANING MOTOR SETTINGS =====
int brushA_pwm = 0;
int brushB_pwm = 0;
bool brushA_enabled = true;
bool brushB_enabled = true;

// ===== SAFETY =====
unsigned long lastCommandTime = 0;
const unsigned long TIMEOUT_MS = 1000;  // Stop if no command for 1 second

// ===== COMMUNICATION =====
const unsigned long BAUD_RATE = 115200;

void setup() {
  Serial.begin(BAUD_RATE);
  
  // Configure actuator pins
  pinMode(ACT1_PWM, OUTPUT);
  pinMode(ACT1_DIR, OUTPUT);
  pinMode(ACT2_PWM, OUTPUT);
  pinMode(ACT2_DIR, OUTPUT);
  
  // Configure cleaning motor pins
  pinMode(BRUSH_A_PWM, OUTPUT);
  pinMode(BRUSH_A_DIR, OUTPUT);
  pinMode(BRUSH_B_PWM, OUTPUT);
  pinMode(BRUSH_B_DIR, OUTPUT);
  
  // Start with everything stopped
  stopAll();
  
  Serial.println("# Combined Actuator & Cleaning Motor Controller Ready");
  Serial.println("# States: IDLE, EXTEND, RETRACT, CLEAN");
  Serial.println("READY");
  
  lastCommandTime = millis();
}

void loop() {
  // Parse commands from Serial
  if (Serial.available()) {
    parseCommand();
    lastCommandTime = millis();
  }
  
  // Safety timeout
  if (millis() - lastCommandTime > TIMEOUT_MS) {
    if (currentState != IDLE) {
      setState(IDLE);
      Serial.println("!TIMEOUT");
    }
  }
  
  // Send status at 10Hz
  static unsigned long lastStatus = 0;
  if (millis() - lastStatus > 100) {
    sendStatus();
    lastStatus = millis();
  }
  
  delay(1);
}

// ===== STATE MANAGEMENT =====
void setState(SystemState newState) {
  if (newState == currentState) return;
  
  currentState = newState;
  
  // Stop everything first
  stopActuators();
  stopBrushes();
  
  // Apply new state
  switch (currentState) {
    case IDLE:
      // Everything already stopped
      break;
      
    case EXTEND:
      extendActuators();
      break;
      
    case RETRACT:
      retractActuators();
      break;
      
    case CLEAN:
      startBrushes();
      break;
  }
  
  Serial.print("STATE:");
  Serial.println(stateNames[currentState]);
}

// ===== ACTUATOR CONTROL =====
void extendActuators() {
  digitalWrite(ACT1_DIR, ACT_EXTEND);
  digitalWrite(ACT2_DIR, ACT_EXTEND);
  analogWrite(ACT1_PWM, actuatorSpeed);
  analogWrite(ACT2_PWM, actuatorSpeed);
}

void retractActuators() {
  digitalWrite(ACT1_DIR, ACT_RETRACT);
  digitalWrite(ACT2_DIR, ACT_RETRACT);
  analogWrite(ACT1_PWM, actuatorSpeed);
  analogWrite(ACT2_PWM, actuatorSpeed);
}

void stopActuators() {
  analogWrite(ACT1_PWM, 0);
  analogWrite(ACT2_PWM, 0);
}

void setActuatorSpeed(int speed) {
  actuatorSpeed = constrain(speed, MIN_ACT_SPEED, MAX_ACT_SPEED);
  
  // Update speed if currently moving
  if (currentState == EXTEND || currentState == CLEAN_EXTEND) {
    extendActuators();
  } else if (currentState == RETRACT || currentState == CLEAN_RETRACT) {
    retractActuators();
  }
}

// ===== CLEANING MOTOR CONTROL =====
void startBrushes() {
  setBrushA(brushA_pwm);
  setBrushB(brushB_pwm);
}

void stopBrushes() {
  setBrushA(0);
  setBrushB(0);
}

void setBrushA(int pwm) {
  pwm = constrain(pwm, -255, 255);
  brushA_pwm = pwm;
  
  if (!brushA_enabled) {
    pwm = 0;
  }
  
  if (pwm >= 0) {
    digitalWrite(BRUSH_A_DIR, LOW);
    analogWrite(BRUSH_A_PWM, pwm);
  } else {
    digitalWrite(BRUSH_A_DIR, HIGH);
    analogWrite(BRUSH_A_PWM, -pwm);
  }
}

void setBrushB(int pwm) {
  pwm = constrain(pwm, -255, 255);
  brushB_pwm = pwm;
  
  if (!brushB_enabled) {
    pwm = 0;
  }
  
  if (pwm >= 0) {
    digitalWrite(BRUSH_B_DIR, LOW);
    analogWrite(BRUSH_B_PWM, pwm);
  } else {
    digitalWrite(BRUSH_B_DIR, HIGH);
    analogWrite(BRUSH_B_PWM, -pwm);
  }
}

// ===== SAFETY =====
void stopAll() {
  stopActuators();
  stopBrushes();
  currentState = IDLE;
}

// ===== COMMAND PARSING =====
void parseCommand() {
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  
  if (cmd.length() == 0) return;
  
  // STATE,<state_name> - Change system state
  if (cmd.startsWith("STATE,")) {
    String state = cmd.substring(6);
    state.toUpperCase();
    
    if (state == "IDLE") {
      setState(IDLE);
      Serial.println("OK:STATE,IDLE");
    }
    else if (state == "EXTEND") {
      setState(EXTEND);
      Serial.println("OK:STATE,EXTEND");
    }
    else if (state == "RETRACT") {
      setState(RETRACT);
      Serial.println("OK:STATE,RETRACT");
    }
    else if (state == "CLEAN") {
      setState(CLEAN);
      Serial.println("OK:STATE,CLEAN");
    }
    else {
      Serial.print("ERR:INVALID_STATE:");
      Serial.println(state);
    }
  }
  
  // ACT_SPEED,xxx - Set actuator speed
  else if (cmd.startsWith("ACT_SPEED,")) {
    int speed = cmd.substring(10).toInt();
    setActuatorSpeed(speed);
    Serial.print("OK:ACT_SPEED,");
    Serial.println(actuatorSpeed);
  }
  
  // CLEAN_SPEED,pwmA,pwmB - Set cleaning motor speeds
  else if (cmd.startsWith("CLEAN_SPEED,")) {
    int comma = cmd.indexOf(',', 12);
    if (comma > 0) {
      int pwmA = cmd.substring(12, comma).toInt();
      int pwmB = cmd.substring(comma + 1).toInt();
      
      brushA_pwm = pwmA;
      brushB_pwm = pwmB;
      
      // Apply if currently cleaning
      if (currentState == CLEAN || currentState == CLEAN_EXTEND || currentState == CLEAN_RETRACT) {
        startBrushes();
      }
      
      Serial.print("OK:CLEAN_SPEED,");
      Serial.print(brushA_pwm);
      Serial.print(",");
      Serial.println(brushB_pwm);
    } else {
      Serial.println("ERR:INVALID_FORMAT");
    }
  }
  
  // S - Emergency stop
  else if (cmd.startsWith("S")) {
    stopAll();
    Serial.println("OK:EMERGENCY_STOP");
  }
  
  // P - Ping
  else if (cmd.startsWith("P")) {
    Serial.println("PONG");
  }
  
  // I - System info
  else if (cmd.startsWith("I")) {
    printInfo();
  }
  
  // Unknown command
  else {
    Serial.print("ERR:UNKNOWN:");
    Serial.println(cmd);
  }
}

// ===== STATUS & INFO =====
void sendStatus() {
  // Format: STATUS,state,actSpeed,brushA,brushB,timestamp
  Serial.print("STATUS,");
  Serial.print(stateNames[currentState]);
  Serial.print(",");
  Serial.print(actuatorSpeed);
  Serial.print(",");
  Serial.print(brushA_pwm);
  Serial.print(",");
  Serial.print(brushB_pwm);
  Serial.print(",");
  Serial.println(millis());
}

void printInfo() {
  Serial.println("# === SYSTEM INFO ===");
  Serial.print("# Current State: ");
  Serial.println(stateNames[currentState]);
  Serial.print("# Actuator Speed: ");
  Serial.println(actuatorSpeed);
  Serial.print("# Brush A PWM: ");
  Serial.print(brushA_pwm);
  Serial.print(", Enabled: ");
  Serial.println(brushA_enabled ? "YES" : "NO");
  Serial.print("# Brush B PWM: ");
  Serial.print(brushB_pwm);
  Serial.print(", Enabled: ");
  Serial.println(brushB_enabled ? "YES" : "NO");
  Serial.print("# Uptime: ");
  Serial.print(millis() / 1000);
  Serial.println(" seconds");
  Serial.println("# ===================");
}