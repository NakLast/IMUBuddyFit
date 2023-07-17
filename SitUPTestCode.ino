#include <Wire.h>

const int MPU_ADDR = 0x68;  // I2C address of the MPU6050
const int LED_PIN = 13;     // Pin connected to an LED (for demonstration purposes)

// Register addresses of MPU6050
const int MPU_REG_ACCEL_X = 0x3B;
const int MPU_REG_ACCEL_Y = 0x3D;
const int MPU_REG_ACCEL_Z = 0x3F;
const int MPU_REG_GYRO_X = 0x43;
const int MPU_REG_GYRO_Y = 0x45;
const int MPU_REG_GYRO_Z = 0x47;

// Constants for sit-up, push-up, and squat detection
const float SITUP_PITCH_THRESHOLD = 30.0;    // Minimum change in pitch angle for sit-up detection
const float PUSHUP_PITCH_THRESHOLD = -30.0;  // Maximum change in pitch angle for push-up detection
const float SQUAT_ROLL_THRESHOLD = 30.0;     // Minimum change in roll angle for squat detection
const int EXERCISE_DELAY = 1000;             // Delay in milliseconds between exercise detections

bool isSitUpDetected = false;
bool isPushUpDetected = false;
bool isSquatDetected = false;
unsigned long lastExerciseTime = 0;

void setup() {
  Wire.begin();
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // Turn off the LED initially

  // Initialize MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Wake up the MPU6050 (out of sleep mode)
  Wire.endTransmission(true);

  Serial.println("MPU6050 initialized");
}

void loop() {
  // Read accelerometer values
  int16_t accelX = readData(MPU_REG_ACCEL_X);
  int16_t accelY = readData(MPU_REG_ACCEL_Y);
  int16_t accelZ = readData(MPU_REG_ACCEL_Z);

  // Read gyroscope values
  int16_t gyroX = readData(MPU_REG_GYRO_X);
  int16_t gyroY = readData(MPU_REG_GYRO_Y);
  int16_t gyroZ = readData(MPU_REG_GYRO_Z);

  // Calculate the roll and pitch angles
  float roll = atan2(accelY, accelZ) * 180 / PI;
  float pitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / PI;

  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print(" Pitch: ");
  Serial.println(pitch);

  // Sit-up detection logic
  if (!isSitUpDetected && abs(pitch) > SITUP_PITCH_THRESHOLD && (millis() - lastExerciseTime) > EXERCISE_DELAY) {
    isSitUpDetected = true;
    lastExerciseTime = millis();
    Serial.println("Sit-up detected!");
    digitalWrite(LED_PIN, HIGH);  // Turn on the LED
  }

  // Push-up detection logic
  if (!isPushUpDetected && pitch < PUSHUP_PITCH_THRESHOLD && (millis() - lastExerciseTime) > EXERCISE_DELAY) {
    isPushUpDetected = true;
    lastExerciseTime = millis();
    Serial.println("Push-up detected!");
    digitalWrite(LED_PIN, HIGH);  // Turn on the LED
  }

  // Squat detection logic
  if (!isSquatDetected && abs(roll) > SQUAT_ROLL_THRESHOLD && (millis() - lastExerciseTime) > EXERCISE_DELAY) {
    isSquatDetected = true;
    lastExerciseTime = millis();
    Serial.println("Squat detected!");
    digitalWrite(LED_PIN, HIGH);  // Turn on the LED
  }

  // Reset exercise detection flags
  if ((isSitUpDetected || isPushUpDetected || isSquatDetected) && abs(pitch) <= SITUP_PITCH_THRESHOLD && abs(roll) <= SQUAT_ROLL_THRESHOLD) {
    isSitUpDetected = false;
    isPushUpDetected = false;
    isSquatDetected = false;
    digitalWrite(LED_PIN, LOW);  // Turn off the LED
  }

  delay(100);
}

int16_t readData(int regAddr) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(regAddr);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, 1);
  return (Wire.read() << 8) | Wire.read();
}
