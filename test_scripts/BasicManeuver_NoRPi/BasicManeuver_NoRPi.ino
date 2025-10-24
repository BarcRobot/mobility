#include <util/atomic.h>

/*------------ VELOCITY PID CLASS ------------*/
class SimpleVelocityPID {
  private:
    float kp, ki, umax;
    float eintegral;
    
  public:
    SimpleVelocityPID() : kp(1), ki(0), umax(255), eintegral(0.0) {}
    
    void setParams(float kpIn, float kiIn, float umaxIn) {
      kp = kpIn; 
      ki = kiIn; 
      umax = umaxIn;
    }
    
    void evalu(float value, float target, float deltaT, int &pwr, int &dir) {
      float e = target - value;
      eintegral = eintegral + e*deltaT;
      
      if(eintegral > 50) eintegral = 50;
      if(eintegral < -50) eintegral = -50;
      
      float u = kp*e + ki*eintegral;
      
      pwr = (int)fabs(u);
      if(pwr > umax) pwr = umax;
      
      dir = 1;
      if(u < 0) dir = -1;
    }
    
    void reset() {
      eintegral = 0.0;
    }
};

/*------------ MECANUM KINEMATICS CLASS ------------*/
class MecanumKinematics {
  private:
    float lx, ly, wheelRadius, countsPerRev, countsPerMeter;
    
  public:
    MecanumKinematics(float lxIn, float lyIn, float rIn, float cprIn) {
      lx = lxIn; ly = lyIn; wheelRadius = rIn; countsPerRev = cprIn;
      countsPerMeter = countsPerRev / (2.0 * PI * wheelRadius);
    }
    
    void inverseKinematics(float vx, float vy, float omega, float wheelVel[4]) {
      wheelVel[0] = (vy - vx - omega * (lx + ly)) * countsPerMeter;
      wheelVel[1] = (vy + vx + omega * (lx + ly)) * countsPerMeter;
      wheelVel[2] = (vy + vx - omega * (lx + ly)) * countsPerMeter;
      wheelVel[3] = (vy - vx + omega * (lx + ly)) * countsPerMeter;
    }
    
    void forwardKinematics(float wheelVel[4], float &vx, float &vy, float &omega) {
      float v[4];
      for(int i = 0; i < 4; i++) {
        v[i] = wheelVel[i] / countsPerMeter;
      }
      
      vx = (v[1] + v[2] - v[0] - v[3]) / 4.0;
      vy = (v[0] + v[1] + v[2] + v[3]) / 4.0;
      omega = (v[1] + v[3] - v[0] - v[2]) / (4.0 * (lx + ly));
    }
};

/*------------ SMOOTH DECELERATION CLASS ------------*/
class SmoothDeceleration {
  private:
    float currentVx, currentVy, currentOmega;
    float decelRate;  // m/s² or rad/s²
    bool isDecelerating;
    
  public:
    SmoothDeceleration() : currentVx(0), currentVy(0), currentOmega(0), 
                           decelRate(0.5), isDecelerating(false) {}
    
    void setDecelRate(float rate) {
      decelRate = rate;
    }
    
    void startDeceleration(float vx, float vy, float omega) {
      currentVx = vx;
      currentVy = vy;
      currentOmega = omega;
      isDecelerating = true;
    }
    
    bool update(float &vx, float &vy, float &omega, float dt) {
      if(!isDecelerating) {
        return false;  // Not decelerating
      }
      
      // Calculate velocity magnitudes
      float velMag = sqrt(currentVx * currentVx + currentVy * currentVy);
      
      // Check if already stopped
      if(velMag < 0.01 && abs(currentOmega) < 0.01) {
        currentVx = 0;
        currentVy = 0;
        currentOmega = 0;
        vx = 0;
        vy = 0;
        omega = 0;
        return true;  // Deceleration complete
      }
      
      // Decelerate linear velocity
      if(velMag > 0.01) {
        float decelAmount = decelRate * dt;
        if(decelAmount >= velMag) {
          // Would overshoot, just stop
          currentVx = 0;
          currentVy = 0;
        } else {
          // Reduce velocity proportionally
          float scale = (velMag - decelAmount) / velMag;
          currentVx *= scale;
          currentVy *= scale;
        }
      } else {
        currentVx = 0;
        currentVy = 0;
      }
      
      // Decelerate angular velocity
      if(abs(currentOmega) > 0.01) {
        float angDecelAmount = decelRate * dt;  // Using same rate
        if(angDecelAmount >= abs(currentOmega)) {
          currentOmega = 0;
        } else {
          if(currentOmega > 0) {
            currentOmega -= angDecelAmount;
          } else {
            currentOmega += angDecelAmount;
          }
        }
      } else {
        currentOmega = 0;
      }
      
      // Output current velocities
      vx = currentVx;
      vy = currentVy;
      omega = currentOmega;
      
      return false;  // Still decelerating
    }
    
    bool isActive() {
      return isDecelerating;
    }
    
    void stop() {
      isDecelerating = false;
    }
};

/*------------ PIN DEFINITIONS ------------*/
#define NMOTORS 4

const int enca[] = {18, 19, 20, 21};
const int encb[] = {30, 31, 32, 33};
const int pwm[] = {9, 10, 5, 3};
const int ina[] = {2, 7, 22, 25};
const int inb[] = {4, 8, 23, 26};
const int en[] = {6, 12, 24, 27};

const float lx = 0.355;
const float ly = 0.190;
const float wheelRadius = 0.05;
const float countsPerRev = 134.4;

/*------------ GLOBALS ------------*/
long prevT = 0;
int posPrev[] = {0, 0, 0, 0};
volatile int posi[] = {0, 0, 0, 0};

float v1Filt[] = {0, 0, 0, 0};
float v1Prev[] = {0, 0, 0, 0};

SimpleVelocityPID pid[NMOTORS];
MecanumKinematics kinematics(lx, ly, wheelRadius, countsPerRev);
SmoothDeceleration decel;

// Test pattern variables
float targetVx = 0.0;
float targetVy = 0.0;
float targetOmega = 0.0;

// Odometry
float odomX = 0.0;
float odomY = 0.0;
float odomTheta = 0.0;

// Test state machine
enum TestState { WAIT_START, FORWARD, BACKWARD, DECELERATING, STOPPED };
TestState testState = WAIT_START;
int cycleCount = 0;
unsigned long stateStartTime = 0;

bool firstLoop = true;

/*------------ FUNCTIONS ------------*/
void setMotor(int dir, int pwmVal, int pwm, int ina, int inb) {
  analogWrite(pwm, pwmVal);
  if(dir == 1) {
    digitalWrite(ina, HIGH); digitalWrite(inb, LOW);
  } else if(dir == -1) {
    digitalWrite(ina, LOW); digitalWrite(inb, HIGH);
  } else {
    digitalWrite(ina, LOW); digitalWrite(inb, LOW);
  }
}

template <int j>
void readEncoder() {
  int b = digitalRead(encb[j]);
  if(b > 0) posi[j]++;
  else posi[j]--;
}

void sendTelemetry(float actualVx, float actualVy, float actualOmega) {
  Serial.print("T,");
  Serial.print(actualVx, 3); Serial.print(",");
  Serial.print(actualVy, 3); Serial.print(",");
  Serial.print(actualOmega, 3); Serial.print(",");
  Serial.print(odomX, 3); Serial.print(",");
  Serial.print(odomY, 3); Serial.print(",");
  Serial.print(odomTheta, 3); Serial.print(",");
  Serial.print(cycleCount); Serial.print(",");
  Serial.print(testState); Serial.print(",");
  Serial.println(millis());
}

void updateTestPattern(float actualVx, float actualVy, float actualOmega, float dt) {
  unsigned long currentTime = millis();
  unsigned long elapsed = currentTime - stateStartTime;
  
  switch(testState) {
    case WAIT_START:
      // Wait 3 seconds before starting
      targetVx = 0;
      targetVy = 0;
      targetOmega = 0;
      
      if(elapsed > 3000) {
        Serial.println("\n=== STARTING TEST PATTERN ===");
        Serial.println("Pattern: 2 cycles of forward/backward at 0.1 m/s");
        Serial.println("Cycle duration: 10 seconds each direction");
        Serial.println("Deceleration rate: 0.5 m/s²");
        testState = FORWARD;
        stateStartTime = currentTime;
        cycleCount = 1;
      }
      break;
      
    case FORWARD:
      targetVx = 0;
      targetVy = 0.1;  // Forward at 0.1 m/s
      targetOmega = 0;
      
      if(elapsed > 10000) {  // 10 seconds = 1 meter
        Serial.print("Cycle ");
        Serial.print(cycleCount);
        Serial.println(" - Forward complete, switching to backward");
        testState = BACKWARD;
        stateStartTime = currentTime;
      }
      break;
      
    case BACKWARD:
      targetVx = 0;
      targetVy = -0.1;  // Backward at 0.1 m/s
      targetOmega = 0;
      
      if(elapsed > 10000) {  // 10 seconds = 1 meter
        Serial.print("Cycle ");
        Serial.print(cycleCount);
        Serial.println(" - Backward complete");
        
        cycleCount++;
        
        if(cycleCount <= 2) {  // Changed to 2 cycles
          // Continue to next cycle
          testState = FORWARD;
          stateStartTime = currentTime;
        } else {
          // Start smooth deceleration
          Serial.println("\n=== STARTING SMOOTH DECELERATION ===");
          Serial.print("Current velocity: vx=");
          Serial.print(actualVx, 3);
          Serial.print(" m/s, vy=");
          Serial.print(actualVy, 3);
          Serial.println(" m/s");
          Serial.println("Decelerating at 0.5 m/s²...");
          
          decel.setDecelRate(0.5);  // 0.5 m/s² deceleration
          decel.startDeceleration(actualVx, actualVy, actualOmega);
          testState = DECELERATING;
          stateStartTime = currentTime;
        }
      }
      break;
      
    case DECELERATING:
      // Smooth deceleration updates target velocity
      bool decelComplete = decel.update(targetVx, targetVy, targetOmega, dt);
      
      if(decelComplete) {
        Serial.println("\n=== DECELERATION COMPLETE ===");
        Serial.print("Time to stop: ");
        Serial.print((currentTime - stateStartTime) / 1000.0, 2);
        Serial.println(" seconds");
        Serial.println("\n=== TEST COMPLETE ===");
        Serial.println("Final odometry position:");
        Serial.print("  X: "); Serial.print(odomX, 3); Serial.println(" m");
        Serial.print("  Y: "); Serial.print(odomY, 3); Serial.println(" m");
        Serial.print("  Theta: "); Serial.print(odomTheta * 180 / PI, 1); Serial.println(" degrees");
        
        // Calculate return-to-start error
        float errorDist = sqrt(odomX * odomX + odomY * odomY);
        Serial.print("\nReturn-to-start error: ");
        Serial.print(errorDist * 100, 1);
        Serial.println(" cm");
        
        Serial.println("\nAll motors stopped. Reset Arduino to run test again.");
        testState = STOPPED;
        
        // Disable motors completely
        for(int k = 0; k < NMOTORS; k++) {
          setMotor(0, 0, pwm[k], ina[k], inb[k]);
        }
      }
      break;
      
    case STOPPED:
      // Stay stopped - don't send any commands
      // Motors are already disabled
      break;
  }
}

/*------------ SETUP ------------*/
void setup() {
  Serial.begin(115200);
  
  for(int k = 0; k < NMOTORS; k++) {
    pinMode(enca[k], INPUT);
    pinMode(encb[k], INPUT);
    pinMode(pwm[k], OUTPUT);
    pinMode(ina[k], OUTPUT);
    pinMode(inb[k], OUTPUT);
    pinMode(en[k], OUTPUT);
    digitalWrite(en[k], HIGH);
    pid[k].setParams(5, 10, 255);
    setMotor(0, 0, pwm[k], ina[k], inb[k]);
  }
  
  attachInterrupt(digitalPinToInterrupt(enca[0]), readEncoder<0>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]), readEncoder<1>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[2]), readEncoder<2>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[3]), readEncoder<3>, RISING);
  
  Serial.println("=== Forward/Backward Test with Smooth Deceleration ===");
  Serial.println("Test will begin in 3 seconds...");
  Serial.println();
  
  stateStartTime = millis();
}

/*------------ LOOP ------------*/
void loop() {
  
  // Read encoder positions
  int pos[NMOTORS];
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    for(int k = 0; k < NMOTORS; k++) pos[k] = posi[k];
  }
  
  // Calculate time step
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6;
  
  if(firstLoop) {
    prevT = currT;
    for(int k = 0; k < NMOTORS; k++) posPrev[k] = pos[k];
    firstLoop = false;
    return;
  }
  prevT = currT;
  
  // Calculate wheel velocities
  float velocity1[NMOTORS];
  for(int k = 0; k < NMOTORS; k++) {
    velocity1[k] = (pos[k] - posPrev[k]) / deltaT;
    posPrev[k] = pos[k];
  }
  
  // Filter velocities
  for(int k = 0; k < NMOTORS; k++) {
    v1Filt[k] = 0.854*v1Filt[k] + 0.0728*velocity1[k] + 0.0728*v1Prev[k];
    v1Prev[k] = velocity1[k];
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
  
  // Update test pattern (includes deceleration logic)
  updateTestPattern(actualVx, actualVy, actualOmega, deltaT);
  
  // Only run PID if not stopped
  if(testState != STOPPED) {
    // Calculate target wheel velocities
    float wheelVelTargets[NMOTORS];
    kinematics.inverseKinematics(targetVx, targetVy, targetOmega, wheelVelTargets);
    
    // PID control
    for(int k = 0; k < NMOTORS; k++) {
      int pwr, dir;
      pid[k].evalu(v1Filt[k], wheelVelTargets[k], deltaT, pwr, dir);
      setMotor(dir, pwr, pwm[k], ina[k], inb[k]);
    }
  }
  
  // Send telemetry at 50Hz
  static unsigned long lastTelem = 0;
  if(currT - lastTelem > 20000) {
    sendTelemetry(actualVx, actualVy, actualOmega);
    lastTelem = currT;
  }
  
  delayMicroseconds(100);
}