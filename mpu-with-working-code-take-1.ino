#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <LiquidCrystal.h>

// LCD Pin Definitions
const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// IR Sensor Pin Definitions
const int rightIrSensorPin = A2; // Right IR sensor
const int leftIrSensorPin = A1;  // Left IR sensor
const int encoder = 10;          // Encoder pin

// Motor Pin Definitions
const int IN1 = 1, IN2 = 2, IN3 = 12, IN4 = 13;
const int ENA = 3, ENB = 11;

// MPU6050
Adafruit_MPU6050 mpu;

// State Variables
String currentCommand = "";
bool carMoving = false;
bool rampDetected = false;
bool atopRamp = false;

unsigned long startTime = 0;
unsigned long elapsedTime = 0;
float distance = 0.0;
int previous_state = 0;

// Wheel and Encoder Parameters
const float WHEEL_RADIUS = 3.45; // Radius of the wheel in cm
const int COUNTS_PER_REV = 20;  // Encoder counts per revolution
volatile int oneCount1 = 0;

// Function Prototypes
void updateCommand(String command);
void displayTotalTime();
void moveForward();
void pivotRight();
void pivotLeft();
void stopMotors();
void spin360();
void displayDistance();
void displayTime(unsigned long time);
float calculateAngle();

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

  // Initialize MPU6050
  if (!mpu.begin()) {
    lcd.print("MPU not found!");
    while (1) {
      delay(10);
    }
  }
  lcd.print("MPU Ready");
  delay(1000);
  lcd.clear();
}

void loop() {
  // Read IR sensor values
  int leftIrValue = digitalRead(leftIrSensorPin);
  int rightIrValue = digitalRead(rightIrSensorPin);
  int current_state = digitalRead(encoder);

  // Measure ramp angle
  float angle = calculateAngle();

  // Control logic for movement
  if (!rampDetected && angle > 5.0) { // Ramp detected at angle > 5 degrees
    rampDetected = true;
    lcd.print("Ramp detected");
    delay(1000);
    lcd.clear();
  }

  if (rampDetected && !atopRamp && angle < 5.0) { // At the top of the ramp
    atopRamp = true;
    stopMotors();
    lcd.print("At top of ramp");
    delay(4000); // Stop for 4 seconds
    spin360();
  }

  if (!rampDetected) {
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
      if (carMoving) {
        stopMotors();
        carMoving = false;
        displayTotalTime();
      }
    }
  }

  // Update distance and time if the car is moving
  if (carMoving) {
    elapsedTime = millis() - startTime;
    distance = (oneCount1 * (2 * PI * WHEEL_RADIUS)) / COUNTS_PER_REV; // Distance in cm
    displayTime(elapsedTime);
    displayDistance();
  }
}

// Calculate the pitch angle using MPU6050
float calculateAngle() {
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // Calculate pitch angle
  float pitch = atan2(accel.acceleration.x, sqrt(accel.acceleration.y * accel.acceleration.y + accel.acceleration.z * accel.acceleration.z)) * 180 / PI;
  return pitch;
}

// Update LCD with the current command
void updateCommand(String command) {
  if (command != currentCommand) {
    currentCommand = command;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Cmd: ");
    lcd.print(command);
  }
}

// Display total time when the car stops
void displayTotalTime() {
  unsigned long totalSeconds = elapsedTime / 1000;
  unsigned long minutes = totalSeconds / 60;
  unsigned long seconds = totalSeconds % 60;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("TimeT: ");
  lcd.print(minutes);
  lcd.print("m ");
  if (seconds < 10) {
    lcd.print("0");
  }
  lcd.print(seconds);
  lcd.print("s ");
}

// Motor control functions
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
}

void spin360() {
  lcd.clear();
  lcd.print("Spinning 360");
  analogWrite(ENA, 200);
  analogWrite(ENB, 200);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(2000); // Adjust based on the time it takes to spin
  stopMotors();
}

// Display distance on LCD
void displayDistance() {
  lcd.setCursor(0, 1);
  lcd.print("Dist: ");
  lcd.print(distance, 1);
  lcd.print(" cm ");
}

// Display time on LCD
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
