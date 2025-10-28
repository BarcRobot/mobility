#include <util/atomic.h>

/*------------ ZERO-JITTER TUNING ------------*/
// Minimal effective PWM to overcome static friction
const int   PWM_MIN_MOVE    = 20;      // tweak 20–30 if starts are sticky
// Limit how fast PWM can change each loop (smoothing)
const int   PWM_SLEW_STEP   = 10;      // tweak 5–15
// Integrator bleed when hovering near zero error
const float INT_DECAY_NEAR0 = 0.90f;
/*--------------------------------------------*/

// Robot geometry (measure your robot!)
const float lx           = 0.355f;   // half wheelbase (m)
const float ly           = 0.190f;   // half track width (m)
const float wheelRadius  = 0.05f;    // wheel radius (m) (100 mm dia)
const float countsPerRev = 134.4f;   // encoder counts per wheel revolution

// Current monitoring
const float CURRENT_WARNING = 6.0;   // Amps - warning threshold
const float CURRENT_CRITICAL = 10.0; // Amps - emergency stop threshold

// Strict robot-level deadbands
const float VLIN_DEADBAND = 0.08f;   // m/s: treat |v| < 0.10 as zero
const float VANG_DEADBAND = 0.08f;   // rad/s: treat |ω| < 0.20 as zero

/*------------ VELOCITY PID CLASS ------------*/
class VelocityPID{
  private:
    float kp, ki, umax;   // Parameters
    float eintegral;      // Storage

    static float clampf(float x, float lo, float hi){
      if(x < lo) return lo;
      if(x > hi) return hi;
      return x;
    }

  public:
  VelocityPID() : kp(1), ki(0), umax(255), eintegral(0.0f) {}

  void setParams(float kpIn, float kiIn, float umaxIn){
    kp = kpIn; ki = kiIn; umax = umaxIn;
  }

  // Global, stricter per-wheel RPM deadband (set in setup)
  static float RPM_DEADBAND_G;

  // Compute control signal (PI with anti-windup, deadband, minimum PWM)
  void evalu(int value, int target, float deltaT, int &pwr, int &dir){
    // --- Wheel-space deadband: if both |target| and |measured| are tiny, stop cleanly
    if (fabsf(target) < RPM_DEADBAND_G && fabsf(value) < RPM_DEADBAND_G){
      pwr = 0; dir = 0;
      eintegral = 0.0f; // flush integrator to avoid kick when leaving zero
      return;
    }

    // Error
    float e = target - value;

    // Integrate with anti-windup clamp
    eintegral += e * deltaT;
    if (ki > 1e-6f){
      float iMax = umax / ki;
      eintegral = clampf(eintegral, -iMax, iMax);
    }

    // Gentle decay when near zero error
    if (fabsf(e) < RPM_DEADBAND_G){
      eintegral *= INT_DECAY_NEAR0;
    }

    // PI output
    float u = kp*e + ki*eintegral;

    // Magnitude and saturation
    pwr = (int)fabsf(u);
    if (pwr > (int)umax) pwr = (int)umax;

    // Minimum effective PWM to overcome stiction (only if nonzero command)
    if (pwr > 0 && pwr < PWM_MIN_MOVE){
      pwr = PWM_MIN_MOVE;
    }

    // Direction
    dir = (u < 0.0f) ? -1 : 1;
  }

  void reset() { eintegral = 0.0f; }
};

// Define static
float VelocityPID::RPM_DEADBAND_G = 0.0f;

/*------------ VELOCITY FILTER CLASS ------------*/
class VelocityFilter {
  private:
    int   posPrev;
    float v1Filt;
    float v1Prev;
    
  public:
    VelocityFilter() : posPrev(0), v1Filt(0.0f), v1Prev(0.0f) {}
    
    // Method 1: velocity from position difference
    float getVelocity1(int pos, float deltaT) {
      float velocity1 = (pos - posPrev) / deltaT; // counts/s
      posPrev = pos;
      return velocity1;
    }
    
    // IIR filter (~25 Hz cutoff at typical loop rates)
    float filterVelocity1(float v1) {
      v1Filt = 0.854f * v1Filt + 0.0728f * v1 + 0.0728f * v1Prev;
      v1Prev = v1;
      return v1Filt;
    }
    
    float getFilteredVel1() const { return v1Filt; }
};

/*------------ MECANUM KINEMATICS CLASS ------------*/
class MecanumKinematics {
  private:
    float lx, ly;              // Robot geometry (meters)
    float countsPerRev;        // Encoder counts per wheel revolution
    float wheelRadius;         // Wheel radius (meters)
    
  public:
    MecanumKinematics(float lxIn, float lyIn, float rIn, float cprIn)
    : lx(lxIn), ly(lyIn), countsPerRev(cprIn), wheelRadius(rIn) {}
    
    // Robot velocity → Wheel RPM (inverse kinematics)
    void inverseKinematics(float vx, float vy, float omega, float wheelRPM[4]) {
      const float L = (lx + ly);
      const float v0_ms =  vy - vx - omega * L;  // Front-left
      const float v1_ms =  vy + vx + omega * L;  // Front-right
      const float v2_ms =  vy + vx - omega * L;  // Rear-left
      const float v3_ms =  vy - vx + omega * L;  // Rear-right
      
      const float wheelCircum = 2.0f * PI * wheelRadius;
      wheelRPM[0] =  (v0_ms / wheelCircum) * 60.0f;
      wheelRPM[1] = -(v1_ms / wheelCircum) * 60.0f; // right side negated
      wheelRPM[2] =  (v2_ms / wheelCircum) * 60.0f;
      wheelRPM[3] = -(v3_ms / wheelCircum) * 60.0f; // right side negated
    }
    
    // Wheel RPM → Robot velocity (forward kinematics)
    void forwardKinematics(float wheelRPM[4], float &vx, float &vy, float &omega) {
      const float wheelCircum = 2.0f * PI * wheelRadius;
      float v[4];
      for(int i = 0; i < 4; i++) v[i] = (wheelRPM[i] / 60.0f) * wheelCircum;
      vx    = (v[1] + v[2] - v[0] - v[3]) / 4.0f;
      vy    = (v[0] + v[1] + v[2] + v[3]) / 4.0f;
      omega = (v[1] + v[3] - v[0] - v[2]) / (4.0f * (lx + ly));
    }
    
    float getCountsPerRev() const { return countsPerRev; }
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

// Encoder & driver pins
const int enca[] = {18, 19, 20, 21};
const int encb[] = {34, 35, 36, 37};
const int pwm[]  = {9, 10, 5, 3};
const int ina[]  = {2, 7, 22, 25};
const int inb[]  = {4, 8, 23, 26};

// Globals
long prevT = 0;
volatile long posi[] = {0,0,0,0}; // encoder positions

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
float targetVx    = 0.0f;  // m/s
float targetVy    = 0.0f;  // m/s
float targetOmega = 0.0f;  // rad/s
unsigned long lastCommandTime = 0;

// Odometry
float odomX = 0.0;
float odomY = 0.0;
float odomTheta = 0.0;

// Current readings
float motorCurrents[NMOTORS] = {0, 0, 0, 0};
float totalCurrent = 0.0;

unsigned long lastCurrentWarning = 0;

// Status
enum Status { OK, WARN, ERROR, ESTOP };
Status systemStatus = OK;

bool firstLoop = true;

// For change-detection printing
float lastVx = 1e9f, lastVy = 1e9f, lastOmega = 1e9f; // force first valid print
const float EPS = 1e-4f; // minimal change to count as "updated"

// PID, filters, kinematics
VelocityPID   pid[NMOTORS];
VelocityFilter velFilter[NMOTORS];
MecanumKinematics kinematics(lx, ly, wheelRadius, countsPerRev);

// Slew state
int lastPwmCmd[NMOTORS] = {0,0,0,0};
int lastDirCmd[NMOTORS] = {0,0,0,0};

/*------------ FORWARD DECLS ------------*/
template <int j> void readEncoder();
void setMotor(int idx, int dir, int pwmVal, int pwmPin, int inaPin, int inbPin);
void parseCommand();

void setup() {
  Serial.begin(115200);

  // Map 0.10 m/s robot deadband to per-wheel RPM deadband
  {
    const float wheelCircum = 2.0f * PI * wheelRadius;
    VelocityPID::RPM_DEADBAND_G = (VLIN_DEADBAND / wheelCircum) * 60.0f; // ≈ 19 RPM for r=0.05
  }

  for(int k = 0; k < NMOTORS; k++){
    // Encoder pins
    pinMode(enca[k], INPUT);
    pinMode(encb[k], INPUT);

    // Motor driver pins
    pinMode(pwm[k],  OUTPUT);
    pinMode(ina[k],  OUTPUT);
    pinMode(inb[k],  OUTPUT);

    // Initialize PID
    pid[k].setParams(5.0f, 10.0f, 255.0f); // kp, ki, umax

    // Initialize current monitors
    currentMon[k].begin();
  }

  // Correct attachInterrupt calls (templates use angle brackets)
  attachInterrupt(digitalPinToInterrupt(enca[M0]), readEncoder<M0>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[M1]), readEncoder<M1>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[M2]), readEncoder<M2>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[M3]), readEncoder<M3>, RISING);

  prevT = micros();

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
  // Atomic snapshot of encoders
  int pos[NMOTORS];
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    for(int k = 0; k < NMOTORS; k++) pos[k] = posi[k];
  }

  // deltaT
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6f;
  prevT = currT;

  // Velocity estimation & filtering
  float velocity1[NMOTORS];
  float v1_rpm[NMOTORS];
  float v1_filt[NMOTORS];
  for(int k = 0; k < NMOTORS; k++) {
    velocity1[k] = velFilter[k].getVelocity1(pos[k], deltaT);   // counts/s
    v1_rpm[k]    = velocity1[k] / countsPerRev * 60.0f;         // RPM
    v1_filt[k]   = velFilter[k].filterVelocity1(v1_rpm[k]);     // filtered RPM
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

  // Commands from host
  parseCommand(); // may print if targets changed

  // Check safety
  checkSafety();
  
  // Motor control logic
  if(systemStatus == ESTOP) {
    // Emergency stop - motors already set to PWM 0 in parseCommand()
    // Keep them at zero
    for(int k = 0; k < NMOTORS; k++) {
      setMotor(k, 0, 0, pwm[k], ina[k], inb[k]);
    }
  }

  // Inverse kinematics -> target wheel RPM
  float targetRPM[NMOTORS];
  kinematics.inverseKinematics(targetVx, targetVy, targetOmega, targetRPM);

  // 4x PI control
  for(int k = 0; k < NMOTORS; k++){
    int pwr, dir;
    pid[k].evalu((int)v1_filt[k], (int)targetRPM[k], deltaT, pwr, dir);
    setMotor(k, dir, pwr, pwm[k], ina[k], inb[k]);
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

  // Keep loop snappy but not busy-waiting
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
void setMotor(int idx, int dir, int pwmVal, int pwmPin, int inaPin, int inbPin) {
  // If direction flips, force a brief brake (drop PWM this cycle)
  if (dir != lastDirCmd[idx]){
    lastPwmCmd[idx] = 0;
    analogWrite(pwmPin, 0);
    digitalWrite(inaPin, LOW);
    digitalWrite(inbPin, LOW);
    lastDirCmd[idx] = dir;
  }

  // Slew-limit PWM toward desired
  int desiredPwm = pwmVal;
  int diff = desiredPwm - lastPwmCmd[idx];
  if (diff >  PWM_SLEW_STEP)      lastPwmCmd[idx] += PWM_SLEW_STEP;
  else if (diff < -PWM_SLEW_STEP) lastPwmCmd[idx] -= PWM_SLEW_STEP;
  else                            lastPwmCmd[idx]  = desiredPwm;

  // Apply PWM first
  analogWrite(pwmPin, lastPwmCmd[idx]);

  // Then set H-bridge direction
  if(dir == 1) {
    digitalWrite(inaPin, HIGH);
    digitalWrite(inbPin, LOW);
  } else if(dir == -1) {
    digitalWrite(inaPin, LOW);
    digitalWrite(inbPin, HIGH);
  } else {
    digitalWrite(inaPin, LOW);
    digitalWrite(inbPin, LOW);
  }
}

template <int j>
void readEncoder(){
  int b = digitalRead(encb[j]);
  if(b > 0) posi[j]++;
  else      posi[j]--;
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
      setMotor(k, 0, 0, pwm[k], ina[k], inb[k]);
    }
    
    Serial.println("!!! EMERGENCY STOP DUE TO OVERCURRENT !!!");
    return false;
  }
  
  return true;
}

// Print ONLY when values actually changed.
void parseCommand() {
  if(!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  if(cmd.startsWith("C,")) {
    // Format: "C,vx,vy,omega"
    int comma1 = cmd.indexOf(',', 2);
    int comma2 = cmd.indexOf(',', comma1 + 1);
    if(comma1 > 0 && comma2 > 0) {
      float newVx = cmd.substring(2, comma1).toFloat();
      float newVy = cmd.substring(comma1 + 1, comma2).toFloat();
      float newOm = cmd.substring(comma2 + 1).toFloat();

      // --- Robot-space strict deadbands (snap tiny commands to zero)
      float linMag = sqrtf(newVx*newVx + newVy*newVy);
      if (linMag < VLIN_DEADBAND) { newVx = 0.0f; newVy = 0.0f; }
      if (fabsf(newOm) < VANG_DEADBAND) { newOm = 0.0f; }

      bool changed = (fabsf(newVx - targetVx) > EPS) ||
                     (fabsf(newVy - targetVy) > EPS) ||
                     (fabsf(newOm - targetOmega) > EPS);

      targetVx    = newVx;
      targetVy    = newVy;
      targetOmega = newOm;

      if (changed || fabsf(lastVx-1e9f)<1e-3f) {
        lastVx = targetVx; lastVy = targetVy; lastOmega = targetOmega;
        Serial.print(F("C,"));
        Serial.print(targetVx, 6); Serial.print(',');
        Serial.print(targetVy, 6); Serial.print(',');
        Serial.println(targetOmega, 6);
      }
    }
  }
  else if(cmd.startsWith("S")) {
    bool changed = (fabsf(targetVx) > EPS) || (fabsf(targetVy) > EPS) || (fabsf(targetOmega) > EPS);
    targetVx = 0.0f; targetVy = 0.0f; targetOmega = 0.0f;
    if (changed || fabsf(lastVx-1e9f)<1e-3f) {
      lastVx = targetVx; lastVy = targetVy; lastOmega = targetOmega;
      Serial.println(F("S,0,0,0"));
    }
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
