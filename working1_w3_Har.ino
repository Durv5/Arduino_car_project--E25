// IR sensors ON
#include <Wire.h>
#include <LiquidCrystal.h>
#include <MPU6050.h>

// LCD Pin Definitions
const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// IR Sensor Pin Definitions
const int rightIrSensorPin = A2;
const int leftIrSensorPin = A1;

// Motor Pin Definitions
const int IN1 = 1, IN2 = 2, IN3 = 12, IN4 = 13;
const int ENA = 3, ENB = 11;

// Encoder Pin
const int encoderPin = 10;

// MPU-6050
MPU6050 mpu;
float rampAngle = 0.0;

// Constants
const float WHEEL_RADIUS = 3.45; // cm
const int COUNTS_PER_REV = 20;
const int STOP_DELAY = 4000;
const float MPU_THRESHOLD = 1.6;

// Variables
volatile int encoderCount = 0;
float distanceTravelled = 0.0;

// State Flags
bool IR_ON = true;

// Function Prototypes
void moveBackward(int speed);
void stopMotors();
void rotate360(float rotationSpeed);
void rotate180(float rotationSpeed); 
void alignCar();
void followLine(int distance);
float getMaxAngle(float angles[], int size);

void setup() {
  pinMode(rightIrSensorPin, INPUT);
  pinMode(leftIrSensorPin, INPUT);
  pinMode(encoderPin, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  lcd.begin(16, 2);
  lcd.print("Initializing...");
  delay(2000);  // Longer delay at startup for careful initialization

  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    lcd.clear();
    lcd.print("MPU ERROR");
    while (1);
  }
  lcd.clear();
  delay(500); // Short delay after MPU check
}

void loop() {
  // Move backward until MPU detects ramp
  IR_ON = true;
  alignCar();
  IR_ON = false;
  moveBackward(100);
  delay(2000);  // Added longer delay to ensure smooth backward motion

  // Detect ramp and start increasing speed as we ascend
  for (;;) {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);
    rampAngle = -(atan2(ax, az) * 180 / PI) - 2.78;

    if (rampAngle >= -6.0 && rampAngle <= MPU_THRESHOLD) {
      IR_ON = false; // Turn off IR sensors
      lcd.clear();
      lcd.print("Ramp Detected!");
      delay(1000);  // Longer delay after ramp detection
      break;
    }
  }

  // Ascend the ramp, increasing speed gradually
  float anglesOnRamp[100];  // Array to store angle values
  int angleCount = 0;       // Counter for the array
  for (;;) {
    int speed = map(angleCount, 0, 100, 175, 150); // Increase speed as we ascend
    moveBackward(speed); 
    delay(200);        // Longer delay to ensure the robot moves carefully
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);
    rampAngle = -(atan2(ax, az) * 180 / PI) - 2.78;
    
    // Record angles during incline
    if (angleCount < 100) {
      anglesOnRamp[angleCount] = rampAngle;
      angleCount++;
    }

    if (rampAngle <= -24.8) {
      lcd.clear();
      lcd.print("Mid Incline");
      delay(1000);  // Longer delay after reaching mid incline
      break;
    }
  }

  // Reverse further until specified decline sequence
  for (;;) {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);
    rampAngle = -(atan2(ax, az) * 180 / PI) - 2.78;
    
    moveBackward(100);
    delay(500);  // Longer delay for more careful movement

    // Check for the ramp decline
    if (rampAngle >= -17.0 && rampAngle <= -11.0) {  // Range for -17 to -11
      lcd.clear();
      lcd.print("Approaching Top");
    }
    if (rampAngle >= -11.0 && rampAngle <= 3.0) {  // Range for -11 to 3
      lcd.clear();
      lcd.print("At Top");
      delay(500);
      break;
    }
  }

  stopMotors();
  lcd.clear();
  lcd.print("At Top");
  delay(500); // Longer delay to allow time for visual feedback

  // Turn on IR sensors and align the car
  IR_ON = true;
  alignCar();

  // Calculate the max angle from the recorded values during incline
  float maxAngle = getMaxAngle(anglesOnRamp, angleCount);
  lcd.clear();
  lcd.print("Max Angle: ");
  lcd.setCursor(0, 1);
  lcd.print(maxAngle);
  delay(1000);  // Delay to allow for full display of max angle

  delay(STOP_DELAY); // Wait for 4 seconds before moving on

  // Perform 360° spin at controlled speed
  rotate360(45.0);  // Rotation speed between -30 and -60 degrees/sec

  // Realign car after 360° spin
  alignCar();
  
  // IR sensors turn "OFF"
  IR_ON = false;

  // Descend the ramp with positive rampAngles
  moveBackward(60);
  delay(1000);  // Longer delay for smooth descent

  for (;;) {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);
    rampAngle = -(atan2(ax, az) * 180 / PI) - 2.78;

    if (rampAngle > 10.0 && rampAngle < 30.0) {  // Positive ramp angles
      IR_ON = true; // Turn on IR sensors at bottom
      stopMotors();
      delay(1000);  // Longer delay after stopping at the bottom
      rotate180(45.0);  // Controlled rotation for 180°
      delay(1000);
      break;
    }
  }

  // Follow the line for 250 cm
  followLine(250);
}

// Moves the car backward at the specified speed
void moveBackward(int speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// Stops the motors
void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// Performs a 360° spin with a controlled speed
void rotate360(float rotationSpeed) {
  unsigned long lastTime = millis(); // To track elapsed time
  float dt = 0.0;                    // Time delta for each loop iteration
  float angleZ = 0.0;                // Current angle (in degrees)

  // Motor control: Rotate in place (counter-clockwise or clockwise)
  analogWrite(ENA, 150);
  analogWrite(ENB, 150);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  while (angleZ < 360.0) {
    unsigned long currentTime = millis();
    dt = (currentTime - lastTime) / 1000.0; // Convert to seconds
    lastTime = currentTime;

    // Get the current angular velocity in Z axis from the gyroscope
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);

    // Convert the gyroscope reading (raw value) to degrees per second
    float gyroZ = gz / 131.0; // The scale factor is 131 for the MPU-6050

    // Integrate the angular velocity to get the total angle (in degrees)
    angleZ += gyroZ * dt;

    delay(10); 

    if (angleZ>=360){
      break;
    }
  }

  stopMotors();  // Stop the motors once the target angle is reached
}

// Performs a 180° spin with a controlled speed
void rotate180(float rotationSpeed) {
  unsigned long lastTime = millis(); // To track elapsed time
  float dt = 0.0;                    // Time delta for each loop iteration
  float angleZ = 0.0;                // Current angle (in degrees)

  // Motor control: Rotate in place (counter-clockwise or clockwise)
  analogWrite(ENA, 150);
  analogWrite(ENB, 150);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  while (angleZ < 180.0) {
    unsigned long currentTime = millis();
    dt = (currentTime - lastTime) / 1000.0; // Convert to seconds
    lastTime = currentTime;

    // Get the current angular velocity in Z axis from the gyroscope
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);

    // Convert the gyroscope reading (raw value) to degrees per second
    float gyroZ = gz / 131.0; // The scale factor is 131 for the MPU-6050

    // Integrate the angular velocity to get the total angle (in degrees)
    angleZ += gyroZ * dt;

    delay(10); 
  }

  stopMotors();  // Stop the motors once the target angle is reached
}

// Align the car using the IR sensors
void alignCar() {
  // Code to align the car based on IR sensor values
  int leftSensorValue = analogRead(leftIrSensorPin);
  int rightSensorValue = analogRead(rightIrSensorPin);

  if (leftSensorValue < 400 && rightSensorValue < 400) {
    stopMotors();
    delay(1000);
  }
  // Use the sensor values to correct the car's alignment
}

// Returns the max angle from the angles array
float getMaxAngle(float angles[], int size) {
  float maxAngle = angles[0];
  for (int i = 1; i < size; i++) {
    if (angles[i] > maxAngle) {
      maxAngle = angles[i];
    }
  }
  return maxAngle;
}

// Follows the line for the specified distance (in cm)
void followLine(int distance) {
  while (distanceTravelled < distance) {
    int leftSensorValue = analogRead(leftIrSensorPin);
    int rightSensorValue = analogRead(rightIrSensorPin);

    if (leftSensorValue > rightSensorValue) {
      moveBackward(100);  // Turn left
    } else if (rightSensorValue > leftSensorValue) {
      moveBackward(100);  // Turn right
    } else {
      moveBackward(100);  // Move forward
    }

    distanceTravelled += WHEEL_RADIUS * 2 * PI;  // Update distance based on wheel movement
    delay(100);
  }
}
