#include <Wire.h>
#include <LiquidCrystal.h>
#include <MPU6050.h>

// LCD Pin Definitions
const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// IR Sensor Pin Definitions
const int rightIrSensorPin = A2; // Right IR sensor
const int leftIrSensorPin = A1;  // Left IR sensor
const int encoder = 10;

// Motor Pin Definitions
const int IN1 = 1, IN2 = 2, IN3 = 12, IN4 = 13;
const int ENA = 3, ENB = 11;

// Current Command Tracking
String currentCommand = "";
bool carMoving = false;
unsigned long startTime = 0;
unsigned long elapsedTime = 0;
float distance = 0.0;
int previous_state = 0;

// Wheel and Encoder Parameters
const float WHEEL_RADIUS = 3.45; // Radius of the wheel in cm
const int COUNTS_PER_REV = 20; // Encoder counts per revolution
volatile int oneCount1 = 0;

// Ramp and Gyroscope
MPU6050 mpu;
float rampAngle = 0.0;

// Constants
const float GYRO_OFFSET = -3.71; // Default offset for flat surface
const int STOP_DELAY = 4000;     // Stop duration at the top (in milliseconds)

// State Definitions
enum State { FOLLOW_LINE, ASCEND_RAMP, AT_TOP, DESCEND_RAMP, RESUME_LINE };
State currentState = FOLLOW_LINE;

// Variables for Ramp Handling
bool isOnRamp = false;
unsigned long stopTime = 0;  // Time tracking for delays
bool hasRotated = false;     // Tracks if 360° rotation is completed

// Function Prototypes
void updateCommand(String command);
void moveForward();
void pivotRight();
void pivotLeft();
void stopMotors();
void displayDistance();
void displayTime(unsigned long time);
void detectRamp();
void rotateOnTop();
void handleLineFollowing();

void setup() {
  // Initialize pins
  pinMode(rightIrSensorPin, INPUT);
  pinMode(leftIrSensorPin, INPUT);
  pinMode(encoder, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Initialize LCD
  lcd.begin(16, 2);
  lcd.print("Initializing...");
  delay(1000);
  lcd.clear();

  // Initialize MPU-6050
  Wire.begin();
  mpu.initialize();
  if (mpu.testConnection()) {
    lcd.print("MPU OK");
  } else {
    lcd.print("MPU ERROR");
    while (1); // Halt execution if MPU-6050 is not detected
  }
  delay(1000);
  lcd.clear();
}

void loop() {
  switch (currentState) {
    case FOLLOW_LINE:
      handleLineFollowing();
      if (isOnRamp) {
        currentState = ASCEND_RAMP;
      }
      break;

    case ASCEND_RAMP:
      moveForward();
      detectRamp();
      if (rampAngle > 10.0) { // Threshold for ramp top detection
        currentState = AT_TOP;
        stopMotors();
        stopTime = millis();
      }
      break;

    case AT_TOP:
      if (!hasRotated) {
        if (millis() - stopTime >= STOP_DELAY) {
          rotateOnTop();
          hasRotated = true;
        }
      } else {
        currentState = DESCEND_RAMP;
        isOnRamp = true;
      }
      break;

    case DESCEND_RAMP:
      moveForward();
      detectRamp();
      if (rampAngle < 5.0) { // Threshold for flat surface detection
        isOnRamp = false;
        currentState = RESUME_LINE;
      }
      break;

    case RESUME_LINE:
      handleLineFollowing();
      break;
  }

  // Distance Tracking with Encoder
  int current_state = digitalRead(encoder);
  if (current_state == HIGH && previous_state == LOW) {
    oneCount1++;
  }
  previous_state = current_state;

  // Update Distance and Display
  if (carMoving) {
    elapsedTime = millis() - startTime;
    distance = (oneCount1 * (2 * PI * WHEEL_RADIUS)) / COUNTS_PER_REV;
    displayTime(elapsedTime);
    displayDistance();
  }
}

void updateCommand(String command) {
  if (command != currentCommand) {
    currentCommand = command;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Cmd: ");
    lcd.print(command);
  }
}

void handleLineFollowing() {
  int leftIrValue = digitalRead(leftIrSensorPin);
  int rightIrValue = digitalRead(rightIrSensorPin);

  if (leftIrValue == LOW && rightIrValue == LOW) {
    updateCommand("Forward");
    moveForward();
  } else if (leftIrValue == LOW && rightIrValue == HIGH) {
    updateCommand("Pivot Right");
    pivotRight();
    delay(50);
  } else if (rightIrValue == LOW && leftIrValue == HIGH) {
    updateCommand("Pivot Left");
    pivotLeft();
    delay(50);
  } else {
    stopMotors();
  }
}

void moveForward() {
  analogWrite(ENA, 75);
  analogWrite(ENB, 75);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  if (!carMoving) {
    carMoving = true;
    startTime = millis() - elapsedTime;
  }
}

void pivotRight() {
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void pivotLeft() {
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  carMoving = false;
}

void displayDistance() {
  lcd.setCursor(0, 1);
  lcd.print("Dist: ");
  lcd.print(distance, 1);
  lcd.print(" cm ");
}

void displayTime(unsigned long time) {
  unsigned long seconds = time / 1000;
  unsigned long minutes = seconds / 60;
  seconds %= 60;
  lcd.setCursor(0, 0);
  lcd.print("Time: ");
  lcd.print(minutes);
  lcd.print("m ");
  if (seconds < 10) {
    lcd.print("0");
  }
  lcd.print(seconds);
  lcd.print("s ");
}

void detectRamp() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  rampAngle = atan2(ay, az) * 180 / PI + GYRO_OFFSET;
  lcd.setCursor(0, 1);
  lcd.print("Angle: ");
  lcd.print(rampAngle, 1);
  lcd.print(" ");
}

void rotateOnTop() {
  updateCommand("Rotate");
  analogWrite(ENA, 150);
  analogWrite(ENB, 150);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(4000); // Adjust duration based on motor speed for 360° rotation
  stopMotors();
}


