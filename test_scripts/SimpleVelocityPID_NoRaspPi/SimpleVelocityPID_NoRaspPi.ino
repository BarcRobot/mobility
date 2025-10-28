#include <util/atomic.h>

/*------------ VELOCITY PID CLASS ------------*/
class VelocityPID{
  private:
    float kp, ki, umax; // Parameters
    float eintegral; // Storage

  public:
  // Constructor
  VelocityPID() : kp(1), ki(0), umax(255), eintegral(0.0){}

  // A function to set the parameters
  void setParams(float kpIn, float kiIn, float umaxIn){
    kp = kpIn; ki = kiIn; umax = umaxIn;
  }

  // A function to compute the control signal
  void evalu(int value, int target, float deltaT, int &pwr, int &dir){
    // error
    float e = target - value;
  
    // integral
    eintegral = eintegral + e*deltaT;

    if(eintegral > 50) eintegral = 50;
    if(eintegral < -50) eintegral = -50;
  
    // control signal
    float u = kp*e + ki*eintegral;
  
    // motor power
    pwr = (int) fabs(u);
    if( pwr > umax ){
      pwr = umax;
    }
  
    // motor direction
    dir = 1;
    if(u<0){
      dir = -1;
    }
  }

  void reset() {
    eintegral = 0.0;
  }
};

/*------------ VELOCITY FILTER CLASS ------------*/
class VelocityFilter {
  private:
    int posPrev;
    float v1Filt;
    float v1Prev;
    
  public:
    VelocityFilter() : posPrev(0), v1Filt(0), v1Prev(0) {}
    
    // Method 1: Calculate velocity from position difference
    float getVelocity1(int pos, float deltaT) {
      float velocity1 = (pos - posPrev) / deltaT;
      posPrev = pos;
      return velocity1;
    }
    
    // Filter the velocities (25 Hz cutoff)
    float filterVelocity1(float v1) {
      v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
      v1Prev = v1;
      return v1Filt;
    }
    
    float getFilteredVel1() { return v1Filt; }
};

/*------------ MECANUM KINEMATICS CLASS ------------*/
class MecanumKinematics {
  private:
    float lx, ly;              // Robot geometry (meters)
    float countsPerRev;        // Encoder counts per wheel revolution
    float wheelRadius;         // Wheel radius (meters)
    
  public:
    MecanumKinematics(float lxIn, float lyIn, float rIn, float cprIn) {
      lx = lxIn;
      ly = lyIn;
      wheelRadius = rIn;
      countsPerRev = cprIn;
    }
    
    // Inverse kinematics: Robot velocity → Wheel RPM
    void inverseKinematics(float vx, float vy, float omega, float wheelRPM[4]) {
      // Calculate wheel velocities in m/s
      float v0_ms = vy - vx - omega * (lx + ly);  // Front-left
      float v1_ms = vy + vx + omega * (lx + ly);  // Front-right
      float v2_ms = vy + vx - omega * (lx + ly);  // Rear-left
      float v3_ms = vy - vx + omega * (lx + ly);  // Rear-right
      
      // Convert m/s to RPM
      float wheelCircum = 2.0 * PI * wheelRadius;
      wheelRPM[0] = (v0_ms / wheelCircum) * 60.0;
      wheelRPM[1] = -(v1_ms / wheelCircum) * 60.0;
      wheelRPM[2] = (v2_ms / wheelCircum) * 60.0;
      wheelRPM[3] = -(v3_ms / wheelCircum) * 60.0;
    }
    
    // Forward kinematics: Wheel RPM → Robot velocity
    void forwardKinematics(float wheelRPM[4], float &vx, float &vy, float &omega) {
      // Convert RPM to m/s
      float wheelCircum = 2.0 * PI * wheelRadius;
      float v[4];
      for(int i = 0; i < 4; i++) {
        v[i] = (wheelRPM[i] / 60.0) * wheelCircum;
      }
      
      // Calculate robot velocities
      vx = (v[1] + v[2] - v[0] - v[3]) / 4.0;
      vy = (v[0] + v[1] + v[2] + v[3]) / 4.0;
      omega = (v[1] + v[3] - v[0] - v[2]) / (4.0 * (lx + ly));
    }
    
    float getCountsPerRev() { return countsPerRev; }
};

/*------------ CURRENT MONITOR CLASS ------------*/
class CurrentMonitor {
  private:
    int csPin;
    float currentFilt;
    float alpha;
    float maxCurrent;
    float sumCurrent;
    unsigned long sampleCount;
    
  public:
    CurrentMonitor(int pin) : csPin(pin), currentFilt(0), alpha(0.85), 
                              maxCurrent(0), sumCurrent(0), sampleCount(0) {}
    
    void begin() {
      pinMode(csPin, INPUT);
    }
    
    float readCurrent() {
      // VNH5019 current sense: 140 mV per Amp
      // Arduino ADC: 5V / 1023 = 0.00489 V per count
      // Current (A) = (ADC_value × 0.00489) / 0.14
      
      int rawValue = analogRead(csPin);
      float voltage = rawValue * (5.0 / 1023.0);
      float currentRaw = voltage / 0.14;
      
      // Low-pass filter to reduce noise
      currentFilt = alpha * currentFilt + (1 - alpha) * currentRaw;
      
      // Track statistics
      if(currentFilt > maxCurrent) {
        maxCurrent = currentFilt;
      }
      
      sumCurrent += currentFilt;
      sampleCount++;
      
      return currentFilt;
    }
    
    float getFiltered() { return currentFilt; }
    float getMax() { return maxCurrent; }
    
    float getAverage() { 
      if(sampleCount == 0) return 0;
      return sumCurrent / sampleCount; 
    }
    
    void resetStats() {
      maxCurrent = 0;
      sumCurrent = 0;
      sampleCount = 0;
    }
};

/*------------ GLOBAL DEFINITIONS ------------*/
#define NMOTORS 4
#define M0 0
#define M1 1
#define M2 2
#define M3 3

// Pin definitions
const int enca[] = {18, 19, 20, 21};
const int encb[] = {34, 35, 36, 37};
const int pwm[] = {9, 10, 5, 3};
const int ina[] = {2, 7, 22, 25};
const int inb[] = {4, 8, 23, 26};

// Current sense pins
// Shield 1: A0 (Motor 0), A1 (Motor 1)
// Shield 2: A2 (Motor 2), A3 (Motor 3)
const int csPin[] = {A0, A1, A2, A3};

// Robot geometry (MEASURE THESE FOR YOUR ROBOT!)
const float lx = 0.355;           // Half wheelbase (m)
const float ly = 0.190;           // Half track width (m)
const float wheelRadius = 0.05;   // Wheel radius (m) - 100mm diameter
const float countsPerRev = 134.4; // Encoder counts per wheel revolution

// Current monitoring
const float CURRENT_WARNING = 6.0;   // Amps - warning threshold
const float CURRENT_CRITICAL = 10.0; // Amps - emergency stop threshold

// Globals
long prevT = 0;
volatile long posi[] = {0,0,0,0}; // Encoder positions

// Velocity filtering variables
float v1Filt[] = {0, 0, 0, 0};
float v1Prev[] = {0, 0, 0, 0};

// Current monitors for each motor
CurrentMonitor currentMon[NMOTORS] = {
  CurrentMonitor(A0),
  CurrentMonitor(A1),
  CurrentMonitor(A2),
  CurrentMonitor(A3)
};

// Robot velocity commands
float targetVx = 0.0;      // m/s
float targetVy = 0.0;      // m/s
float targetOmega = 0.0;   // rad/s
unsigned long lastCommandTime = 0;

// Odometry
float odomX = 0.0;
float odomY = 0.0;
float odomTheta = 0.0;

// Current readings
float motorCurrents[NMOTORS] = {0, 0, 0, 0};
float totalCurrent = 0.0;

// Status
enum Status { OK, WARN, ERROR, ESTOP };
Status systemStatus = OK;

// Velocity threshold for zero detection
float velocityThreshold = 0.005;  // m/s - below this is considered "zero"
unsigned long lastCurrentWarning = 0;

bool firstLoop = true;

// PID class instances
VelocityPID pid[NMOTORS];

// Velocity filter instances
VelocityFilter velFilter[NMOTORS];

// Kinematics instance
MecanumKinematics kinematics(lx, ly, wheelRadius, countsPerRev);

void setup() {
  // Serial for USB debugging
  Serial.begin(115200);
  
  for(int k = 0; k < NMOTORS; k++){
    // Encoder pins
    pinMode(enca[k], INPUT);
    pinMode(encb[k], INPUT);
    
    // Motor driver pins
    pinMode(pwm[k], OUTPUT);
    pinMode(ina[k], OUTPUT);
    pinMode(inb[k], OUTPUT);
    
    // Initialize PID
    pid[k].setParams(5, 10, 255); // kp, kd, ki, umax
  }

  // Initialize current monitors
  for(int k = 0; k < NMOTORS; k++) {
    currentMon[k].begin();
  }

  // Start with all motors at PWM 0
  for(int k = 0; k < NMOTORS; k++) {
    setMotor(0, 0, pwm[k], ina[k], inb[k]);
  }
  
  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(enca[M0]), readEncoder<M0>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[M1]), readEncoder<M1>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[M2]), readEncoder<M2>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[M3]), readEncoder<M3>, RISING);

  Serial.println("# Arduino Mecanum Controller with Current Monitoring");
  Serial.println("# Commands:");
  Serial.println("#   C,vx,vy,omega - Set velocity");
  Serial.println("#   S - Emergency stop");
  Serial.println("#   R - Reset odometry");
  Serial.println("#   E - Clear ESTOP");
  Serial.println("#   I - Print current statistics");
  Serial.println("#   Z - Reset current statistics");
  Serial.println("# Current thresholds:");
  Serial.print("#   Warning: ");
  Serial.print(CURRENT_WARNING, 1);
  Serial.println(" A");
  Serial.print("#   Critical: ");
  Serial.print(CURRENT_CRITICAL, 1);
  Serial.println(" A");

  lastCommandTime = millis();
}

void loop() {
  // Read the position in an atomic block to avoid a potential misread
  int pos[NMOTORS];
  int posPrev[NMOTORS];
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    for(int k = 0; k < NMOTORS; k++){
      pos[k] = posi[k];
    }
  }

    // Calculate time step
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6;
  
  if(deltaT < 0.0001 || deltaT > 0.1) {
    prevT = currT;
    return;
  }
  
  if(firstLoop) {
    prevT = currT;
    for(int k = 0; k < NMOTORS; k++) {
      posPrev[k] = pos[k];
    }
    firstLoop = false;
    return;
  }
  prevT = currT;
  
  // Compute velocity with method 1 and filter both methods
  float velocity1[NMOTORS];
  float v1[NMOTORS];
  float v1Filt[NMOTORS];
  
  for(int k = 0; k < NMOTORS; k++) {
    // Velocity method 1
    velocity1[k] = velFilter[k].getVelocity1(pos[k], deltaT);
    
    // Convert count/s to RPM
    v1[k] = velocity1[k] / countsPerRev * 60.0;
    
    // Filter velocities
    v1Filt[k] = velFilter[k].filterVelocity1(v1[k]);
  }

  // Calculate actual robot velocity
  float actualVx, actualVy, actualOmega;
  kinematics.forwardKinematics(v1Filt, actualVx, actualVy, actualOmega);
  
  // Update odometry
  float dx = actualVx * deltaT;
  float dy = actualVy * deltaT;
  float dtheta = actualOmega * deltaT;
  
  odomX += dx * cos(odomTheta) - dy * sin(odomTheta);
  odomY += dx * sin(odomTheta) + dy * cos(odomTheta);
  odomTheta += dtheta;
  
  while(odomTheta > PI) odomTheta -= 2*PI;
  while(odomTheta < -PI) odomTheta += 2*PI;

  // Read motor currents
  readAllCurrents();
  
  // Check current safety (may trigger ESTOP)
  checkCurrentSafety();

  // Parse commands from Raspberry Pi
  // parseCommand();
  
  // Set target using test pattern (comment out when using RPi)
  targetVx = 0.25;
  targetVy = 0;
  // targetVy = 0.5 * (sin(currT/1e6) > 0 ? 1 : -1);  // Square wave
  targetOmega = 0;

  // Check safety
  checkSafety();

  float targetRPM[NMOTORS];
  
  // Motor control logic
  if(systemStatus == ESTOP) {
    // Emergency stop - motors already set to PWM 0 in parseCommand()
    // Keep them at zero
    for(int k = 0; k < NMOTORS; k++) {
      setMotor(0, 0, pwm[k], ina[k], inb[k]);
    }
  }
  else if(isTargetZero()) {
    // Target is zero - just set PWM to 0 (no PID)
    for(int k = 0; k < NMOTORS; k++) {
      setMotor(0, 0, pwm[k], ina[k], inb[k]);
    }
    
    // Reset PID integrators to prevent windup
    for(int k = 0; k < NMOTORS; k++) {
      pid[k].reset();
    }
  }
  else {
    // Non-zero target - run PID control
    
    // Calculate target wheel velocities
    kinematics.inverseKinematics(targetVx, targetVy, targetOmega, targetRPM);
    
    // PID control for each motor
    for(int k = 0; k < NMOTORS; k++) {
      int pwr, dir;
      pid[k].evalu(v1Filt[k], targetRPM[k], deltaT, pwr, dir);
      setMotor(dir, pwr, pwm[k], ina[k], inb[k]);
    }
  }

  // Print for debugging
  Serial.print("Target: ");
  Serial.print(targetVx, 2); Serial.print(" ");
  Serial.print(targetVy, 2); Serial.print(" ");
  Serial.print(targetOmega, 2);
  Serial.print(" | RPM: ");
  for(int k = 0; k < NMOTORS; k++) {
    Serial.print(targetRPM[k], 1); Serial.print(" ");
    Serial.print(v1Filt[k], 1); Serial.print(" ");
  }
  Serial.println();

  delay(1);

  // Send telemetry at 50Hz
  static unsigned long lastTelem = 0;
  if(currT - lastTelem > 20000) {
    sendTelemetry(actualVx, actualVy, actualOmega);
    lastTelem = currT;
  }
  
  delayMicroseconds(100);
}

/*------------ FUNCTIONS ------------*/
void setMotor(int dir, int pwmVal, int pwm, int ina, int inb) {
  analogWrite(pwm, pwmVal);
  
  if(dir == 1) {
    digitalWrite(ina, HIGH);
    digitalWrite(inb, LOW);
  }
  else if(dir == -1) {
    digitalWrite(ina, LOW);
    digitalWrite(inb, HIGH);
  }
  else {
    digitalWrite(ina, LOW);
    digitalWrite(inb, LOW);
  }
}

template <int j>
void readEncoder(){
  int b = digitalRead(encb[j]);
  if(b > 0){
    posi[j]++;
  }
  else{
    posi[j]--;
  }
}

void readAllCurrents() {
  totalCurrent = 0;
  for(int k = 0; k < NMOTORS; k++) {
    motorCurrents[k] = currentMon[k].readCurrent();
    totalCurrent += motorCurrents[k];
  }
}

bool checkCurrentSafety() {
  bool critical = false;
  bool warning = false;
  
  // Check each motor
  for(int k = 0; k < NMOTORS; k++) {
    if(motorCurrents[k] > CURRENT_CRITICAL) {
      Serial.print("!!! CRITICAL: Motor ");
      Serial.print(k);
      Serial.print(" current = ");
      Serial.print(motorCurrents[k], 2);
      Serial.println(" A");
      critical = true;
    }
    else if(motorCurrents[k] > CURRENT_WARNING) {
      warning = true;
    }
  }
  
  // Check total current
  if(totalCurrent > CURRENT_CRITICAL * 2) {
    Serial.print("!!! CRITICAL: Total current = ");
    Serial.print(totalCurrent, 2);
    Serial.println(" A");
    critical = true;
  }
  
  // Print warnings (throttled to once per second)
  if(warning && !critical && (millis() - lastCurrentWarning > 1000)) {
    Serial.print("⚠ WARNING: High current - ");
    for(int k = 0; k < NMOTORS; k++) {
      if(motorCurrents[k] > CURRENT_WARNING) {
        Serial.print("M");
        Serial.print(k);
        Serial.print(":");
        Serial.print(motorCurrents[k], 1);
        Serial.print("A ");
      }
    }
    Serial.println();
    lastCurrentWarning = millis();
  }
  
  // Emergency stop on critical current
  if(critical) {
    targetVx = 0;
    targetVy = 0;
    targetOmega = 0;
    systemStatus = ESTOP;
    
    // Immediately stop all motors
    for(int k = 0; k < NMOTORS; k++) {
      setMotor(0, 0, pwm[k], ina[k], inb[k]);
    }
    
    Serial.println("!!! EMERGENCY STOP DUE TO OVERCURRENT !!!");
    return false;
  }
  
  return true;
}

void parseCommand() {
  if(Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    
    if(cmd.startsWith("C,")) {
      // Command format: C,vx,vy,omega\n
      int idx = 2;
      int comma1 = cmd.indexOf(',', idx);
      int comma2 = cmd.indexOf(',', comma1 + 1);
      
      if(comma1 > 0 && comma2 > 0) {
        targetVx = cmd.substring(idx, comma1).toFloat();
        targetVy = cmd.substring(comma1 + 1, comma2).toFloat();
        targetOmega = cmd.substring(comma2 + 1).toFloat();
        
        lastCommandTime = millis();
        if(systemStatus != ESTOP) {
          systemStatus = OK;
        }
      }
    }
    else if(cmd.startsWith("S")) {
      // Emergency stop
      targetVx = 0;
      targetVy = 0;
      targetOmega = 0;
      systemStatus = ESTOP;
      
      for(int k = 0; k < NMOTORS; k++) {
        setMotor(0, 0, pwm[k], ina[k], inb[k]);
      }
      Serial.println("# EMERGENCY STOP");
    }
    else if(cmd.startsWith("R")) {
      // Reset odometry
      odomX = 0;
      odomY = 0;
      odomTheta = 0;
      Serial.println("# Odometry reset");
    }
    else if(cmd.startsWith("E")) {
      // Clear emergency stop
      if(systemStatus == ESTOP) {
        systemStatus = OK;
        Serial.println("# ESTOP cleared");
      }
    }
    else if(cmd.startsWith("I")) {
      // Request current statistics
      printCurrentStats();
    }
    else if(cmd.startsWith("Z")) {
      // Reset current statistics
      for(int k = 0; k < NMOTORS; k++) {
        currentMon[k].resetStats();
      }
      Serial.println("# Current statistics reset");
    }
  }
}

void printCurrentStats() {
  Serial.println("# === CURRENT STATISTICS ===");
  for(int k = 0; k < NMOTORS; k++) {
    Serial.print("# Motor ");
    Serial.print(k);
    Serial.print(": Current=");
    Serial.print(motorCurrents[k], 2);
    Serial.print("A, Max=");
    Serial.print(currentMon[k].getMax(), 2);
    Serial.print("A, Avg=");
    Serial.print(currentMon[k].getAverage(), 2);
    Serial.println("A");
  }
  Serial.print("# Total: ");
  Serial.print(totalCurrent, 2);
  Serial.println("A");
}

void sendTelemetry(float actualVx, float actualVy, float actualOmega) {
  const char* statusStr[] = {"OK", "WARN", "ERROR", "ESTOP"};
  
  // Format: T,vx,vy,omega,x,y,theta,status,i0,i1,i2,i3,itotal,timestamp
  Serial.print("T,");
  Serial.print(actualVx, 3); Serial.print(",");
  Serial.print(actualVy, 3); Serial.print(",");
  Serial.print(actualOmega, 3); Serial.print(",");
  Serial.print(odomX, 3); Serial.print(",");
  Serial.print(odomY, 3); Serial.print(",");
  Serial.print(odomTheta, 3); Serial.print(",");
  Serial.print(statusStr[systemStatus]); Serial.print(",");
  
  // Motor currents
  for(int k = 0; k < NMOTORS; k++) {
    Serial.print(motorCurrents[k], 2);
    Serial.print(",");
  }
  Serial.print(totalCurrent, 2); Serial.print(",");
  Serial.println(millis());
}

bool checkSafety() {
  // Communication timeout check (1 second)
  if(millis() - lastCommandTime > 1000) {
    if(systemStatus == OK) {
      targetVx = 0;
      targetVy = 0;
      targetOmega = 0;
      systemStatus = ERROR;
      Serial.println("# WARNING: Communication timeout");
    }
    return false;
  }
  return true;
}

bool isTargetZero() {
  // Check if target velocity is essentially zero
  float targetMag = sqrt(targetVx*targetVx + targetVy*targetVy);
  return (targetMag < velocityThreshold && abs(targetOmega) < velocityThreshold);
}

