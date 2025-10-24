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

// Robot geometry (MEASURE THESE FOR YOUR ROBOT!)
const float lx = 0.355;           // Half wheelbase (m)
const float ly = 0.190;           // Half track width (m)
const float wheelRadius = 0.05;   // Wheel radius (m) - 100mm diameter
const float countsPerRev = 134.4; // Encoder counts per wheel revolution

// Globals
long prevT = 0;
volatile long posi[] = {0,0,0,0}; // Encoder positions

// Velocity filtering variables
float v1Filt[] = {0, 0, 0, 0};
float v1Prev[] = {0, 0, 0, 0};

// Robot velocity commands
float targetVx = 0.0;      // m/s
float targetVy = 0.0;      // m/s
float targetOmega = 0.0;   // rad/s

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
  
  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(enca[M0]), readEncoder<M0>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[M1]), readEncoder<M1>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[M2]), readEncoder<M2>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[M3]), readEncoder<M3>, RISING);

  Serial.println("Mecanum velocity control ready");
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

  // Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6;
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

  // Parse commands from Raspberry Pi
  // parseCommand();
  
  // Set target using test pattern (comment out when using RPi)
  targetVx = 0.25;
  targetVy = 0;
  // targetVy = 0.5 * (sin(currT/1e6) > 0 ? 1 : -1);  // Square wave
  targetOmega = 0;

  // Set target wheel velocities using mecanum kinematics
  float targetRPM[NMOTORS];
  kinematics.inverseKinematics(targetVx, targetVy, targetOmega, targetRPM);

  // Control all 4 motors
  for(int k = 0; k < NMOTORS; k++){
    int pwr, dir;
    // Evaluate the control signal using filtered velocity
    pid[k].evalu(v1Filt[k], targetRPM[k], deltaT, pwr, dir);
    
    // Signal the motor
    setMotor(dir, pwr, pwm[k], ina[k], inb[k]);
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

void parseCommand() {
  if(Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    
    if(cmd.startsWith("C,")) {
      // Format: "C,vx,vy,omega"
      int comma1 = cmd.indexOf(',', 2);
      int comma2 = cmd.indexOf(',', comma1 + 1);
      
      if(comma1 > 0 && comma2 > 0) {
        targetVx = cmd.substring(2, comma1).toFloat();
        targetVy = cmd.substring(comma1 + 1, comma2).toFloat();
        targetOmega = cmd.substring(comma2 + 1).toFloat();
      }
    }
    else if(cmd.startsWith("S")) {
      // Stop command
      targetVx = 0;
      targetVy = 0;
      targetOmega = 0;
    }
  }
}
