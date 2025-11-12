/*
 * Simple Brush Motor Controller
 * Receives PWM commands from Raspberry Pi via Serial
 * No current sensing - RPi handles that via battery shunt
 * 
 * Commands:
 *   M,pwmA,pwmB  - Set both motor PWM values (-255 to 255)
 *   A,pwm        - Set motor A only
 *   B,pwm        - Set motor B only
 *   S            - Stop both motors
 *   P            - Ping (returns "PONG")
 */

// Motor A pins
const int PWMA_PIN = 9;
const int DIRA_PIN = 2;

// Motor B pins
const int PWMB_PIN = 10;
const int DIRB_PIN = 3;

// Motor state
int motorA_pwm = 0;
int motorB_pwm = 0;
bool motorA_enabled = true;
bool motorB_enabled = true;

// Safety
unsigned long lastCommandTime = 0;
const unsigned long TIMEOUT_MS = 1000;  // Stop if no command for 1 second

// Communication
const unsigned long BAUD_RATE = 115200;  // Match with RPi

void setup() {
  Serial.begin(BAUD_RATE);
  
  // Configure motor pins
  pinMode(PWMA_PIN, OUTPUT);
  pinMode(DIRA_PIN, OUTPUT);
  pinMode(PWMB_PIN, OUTPUT);
  pinMode(DIRB_PIN, OUTPUT);
  
  // Start with motors stopped
  stopMotors();
  
  // Send ready signal
  Serial.println("# Brush Motor Controller Ready");
  Serial.println("# Waiting for RPi commands...");
  Serial.println("READY");
  
  lastCommandTime = millis();
}

void loop() {
  // Parse commands from Serial
  if(Serial.available()) {
    parseCommand();
    lastCommandTime = millis();  // Reset timeout on any command
  }
  
  // Safety timeout - stop motors if no commands received
  if(millis() - lastCommandTime > TIMEOUT_MS) {
    if(motorA_pwm != 0 || motorB_pwm != 0) {
      stopMotors();
      Serial.println("!TIMEOUT");
    }
  }
  
  // Send heartbeat/status at 10Hz
  static unsigned long lastStatus = 0;
  if(millis() - lastStatus > 100) {
    sendStatus();
    lastStatus = millis();
  }
  
  // Keep loop responsive
  delay(1);
}

void setMotorA(int pwm) {
  // Clamp PWM value
  pwm = constrain(pwm, -255, 255);
  motorA_pwm = pwm;
  
  if(!motorA_enabled) {
    pwm = 0;
  }
  
  // Set direction and PWM
  if(pwm >= 0) {
    digitalWrite(DIRA_PIN, LOW);   // Forward
    analogWrite(PWMA_PIN, pwm);
  } else {
    digitalWrite(DIRA_PIN, HIGH);  // Reverse
    analogWrite(PWMA_PIN, -pwm);
  }
}

void setMotorB(int pwm) {
  // Clamp PWM value
  pwm = constrain(pwm, -255, 255);
  motorB_pwm = pwm;
  
  if(!motorB_enabled) {
    pwm = 0;
  }
  
  // Set direction and PWM
  if(pwm >= 0) {
    digitalWrite(DIRB_PIN, LOW);   // Forward
    analogWrite(PWMB_PIN, pwm);
  } else {
    digitalWrite(DIRB_PIN, HIGH);  // Reverse
    analogWrite(PWMB_PIN, -pwm);
  }
}

void stopMotors() {
  motorA_pwm = 0;
  motorB_pwm = 0;
  setMotorA(0);
  setMotorB(0);
}

void parseCommand() {
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  
  if(cmd.length() == 0) return;
  
  // Command: M,pwmA,pwmB - Set both motors
  if(cmd.startsWith("M,")) {
    int comma = cmd.indexOf(',', 2);
    if(comma > 0) {
      int pwmA = cmd.substring(2, comma).toInt();
      int pwmB = cmd.substring(comma + 1).toInt();
      
      setMotorA(pwmA);
      setMotorB(pwmB);
      
      Serial.print("OK:M,");
      Serial.print(motorA_pwm);
      Serial.print(",");
      Serial.println(motorB_pwm);
    } else {
      Serial.println("ERR:INVALID_FORMAT");
    }
  }
  
  // Command: A,pwm - Set motor A only
  else if(cmd.startsWith("A,")) {
    int pwm = cmd.substring(2).toInt();
    setMotorA(pwm);
    
    Serial.print("OK:A,");
    Serial.println(motorA_pwm);
  }
  
  // Command: B,pwm - Set motor B only
  else if(cmd.startsWith("B,")) {
    int pwm = cmd.substring(2).toInt();
    setMotorB(pwm);
    
    Serial.print("OK:B,");
    Serial.println(motorB_pwm);
  }
  
  // Command: S - Stop both motors
  else if(cmd.startsWith("S")) {
    stopMotors();
    Serial.println("OK:S");
  }
  
  // Command: EA,0/1 - Enable/disable motor A
  else if(cmd.startsWith("EA,")) {
    motorA_enabled = (cmd.substring(3).toInt() != 0);
    if(!motorA_enabled) setMotorA(0);
    
    Serial.print("OK:EA,");
    Serial.println(motorA_enabled ? "1" : "0");
  }
  
  // Command: EB,0/1 - Enable/disable motor B
  else if(cmd.startsWith("EB,")) {
    motorB_enabled = (cmd.substring(3).toInt() != 0);
    if(!motorB_enabled) setMotorB(0);
    
    Serial.print("OK:EB,");
    Serial.println(motorB_enabled ? "1" : "0");
  }
  
  // Command: P - Ping (connection check)
  else if(cmd.startsWith("P")) {
    Serial.println("PONG");
  }
  
  // Command: I - Get info
  else if(cmd.startsWith("I")) {
    printInfo();
  }
  
  // Unknown command
  else {
    Serial.print("ERR:UNKNOWN:");
    Serial.println(cmd);
  }
}

void sendStatus() {
  // Format: S,pwmA,pwmB,enabledA,enabledB,timestamp
  Serial.print("S,");
  Serial.print(motorA_pwm);
  Serial.print(",");
  Serial.print(motorB_pwm);
  Serial.print(",");
  Serial.print(motorA_enabled ? "1" : "0");
  Serial.print(",");
  Serial.print(motorB_enabled ? "1" : "0");
  Serial.print(",");
  Serial.println(millis());
}

void printInfo() {
  Serial.println("# === MOTOR INFO ===");
  Serial.print("# Motor A: PWM=");
  Serial.print(motorA_pwm);
  Serial.print(", Enabled=");
  Serial.println(motorA_enabled ? "YES" : "NO");
  
  Serial.print("# Motor B: PWM=");
  Serial.print(motorB_pwm);
  Serial.print(", Enabled=");
  Serial.println(motorB_enabled ? "YES" : "NO");
  
  Serial.print("# Uptime: ");
  Serial.print(millis() / 1000);
  Serial.println(" seconds");
}