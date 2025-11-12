/*
 * Dual Linear Actuator Test Script
 * 
 * Standalone test script for 2x PA-MC1 linear actuators
 * Tests actuator functionality without Raspberry Pi
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
 * Test Sequence:
 *   1. Extend actuator 1 for 3 seconds
 *   2. Retract actuator 1 for 3 seconds
 *   3. Extend actuator 2 for 3 seconds
 *   4. Retract actuator 2 for 3 seconds
 *   5. Extend both actuators together for 3 seconds
 *   6. Retract both actuators together for 3 seconds
 *   7. Wait 5 seconds and repeat
 */

// Motor driver pin definitions
// Actuator 1
const int PWM_PIN_1 = 5;
const int DIR_PIN_1 = 4;

// Actuator 2
const int PWM_PIN_2 = 6;
const int DIR_PIN_2 = 7;

// Test parameters
const int TEST_SPEED = 200;        // Speed (0-255)
const int MOVE_DURATION = 3000;    // 3 seconds per movement
const int PAUSE_BETWEEN = 1000;    // 1 second pause between tests
const int CYCLE_PAUSE = 5000;      // 5 seconds between full cycles

// Direction definitions
const bool EXTEND = HIGH;
const bool RETRACT = LOW;

void setup() {
  // Initialize serial for debugging
  Serial.begin(9600);
  Serial.println("=================================");
  Serial.println("Dual Actuator Test Script");
  Serial.println("=================================");
  
  // Configure motor driver pins for actuator 1
  pinMode(PWM_PIN_1, OUTPUT);
  pinMode(DIR_PIN_1, OUTPUT);
  
  // Configure motor driver pins for actuator 2
  pinMode(PWM_PIN_2, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);
  
  // Start with motors stopped
  stopActuator1();
  stopActuator2();
  
  Serial.println("Setup complete. Starting test sequence in 3 seconds...");
  delay(3000);
}

void loop() {
  Serial.println("\n--- Starting Test Cycle ---");
  
  // Test 1: Actuator 1 Extend
  Serial.println("\nTest 1: Extending Actuator 1");
  extendActuator1();
  delay(MOVE_DURATION);
  stopActuator1();
  Serial.println("Actuator 1 stopped");
  delay(PAUSE_BETWEEN);
  
  // Test 2: Actuator 1 Retract
  Serial.println("\nTest 2: Retracting Actuator 1");
  retractActuator1();
  delay(MOVE_DURATION);
  stopActuator1();
  Serial.println("Actuator 1 stopped");
  delay(PAUSE_BETWEEN);
  
  // Test 3: Actuator 2 Extend
  Serial.println("\nTest 3: Extending Actuator 2");
  extendActuator2();
  delay(MOVE_DURATION);
  stopActuator2();
  Serial.println("Actuator 2 stopped");
  delay(PAUSE_BETWEEN);
  
  // Test 4: Actuator 2 Retract
  Serial.println("\nTest 4: Retracting Actuator 2");
  retractActuator2();
  delay(MOVE_DURATION);
  stopActuator2();
  Serial.println("Actuator 2 stopped");
  delay(PAUSE_BETWEEN);
  
  // Test 5: Both Actuators Extend
  Serial.println("\nTest 5: Extending BOTH Actuators");
  extendActuator1();
  extendActuator2();
  delay(MOVE_DURATION);
  stopActuator1();
  stopActuator2();
  Serial.println("Both actuators stopped");
  delay(PAUSE_BETWEEN);
  
  // Test 6: Both Actuators Retract
  Serial.println("\nTest 6: Retracting BOTH Actuators");
  retractActuator1();
  retractActuator2();
  delay(MOVE_DURATION);
  stopActuator1();
  stopActuator2();
  Serial.println("Both actuators stopped");
  
  // Wait before next cycle
  Serial.println("\n--- Test Cycle Complete ---");
  Serial.print("Waiting ");
  Serial.print(CYCLE_PAUSE / 1000);
  Serial.println(" seconds before next cycle...");
  delay(CYCLE_PAUSE);
}

// Actuator 1 control functions
void extendActuator1() {
  digitalWrite(DIR_PIN_1, EXTEND);
  analogWrite(PWM_PIN_1, TEST_SPEED);
}

void retractActuator1() {
  digitalWrite(DIR_PIN_1, RETRACT);
  analogWrite(PWM_PIN_1, TEST_SPEED);
}

void stopActuator1() {
  analogWrite(PWM_PIN_1, 0);
}

// Actuator 2 control functions
void extendActuator2() {
  digitalWrite(DIR_PIN_2, EXTEND);
  analogWrite(PWM_PIN_2, TEST_SPEED);
}

void retractActuator2() {
  digitalWrite(DIR_PIN_2, RETRACT);
  analogWrite(PWM_PIN_2, TEST_SPEED);
}

void stopActuator2() {
  analogWrite(PWM_PIN_2, 0);
}

// Emergency stop function (stops everything)
void emergencyStop() {
  stopActuator1();
  stopActuator2();
  Serial.println("!!! EMERGENCY STOP !!!");
}