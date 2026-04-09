#include <Wire.h>
#include <MPU6050.h>

MPU6050 imu;

// Raw data
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Converted
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;

// Angles
float pitch = 0, roll = 0;
float pitchAcc, rollAcc;

// Timing
unsigned long prevTime;
float dt;

// Filter constant
float alpha = 0.98;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  imu.initialize();

  if (!imu.testConnection()) {
    Serial.println("MPU FAILED");
    while (1);
  }

  Serial.println("MPU READY");
  prevTime = millis();
}

void loop() {
  // Time
  unsigned long currentTime = millis();
  dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  // Read data
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert
  accX = ax / 16384.0;
  accY = ay / 16384.0;
  accZ = az / 16384.0;

  gyroX = gx / 131.0;
  gyroY = gy / 131.0;
  gyroZ = gz / 131.0;

  // Accelerometer angles
  pitchAcc = atan2(accY, accZ) * 180 / PI;
  rollAcc  = atan2(-accX, accZ) * 180 / PI;

  // Complementary filter
  pitch = alpha * (pitch + gyroX * dt) + (1 - alpha) * pitchAcc;
  roll  = alpha * (roll  + gyroY * dt) + (1 - alpha) * rollAcc;

  // Output
  Serial.print("Pitch: ");
  Serial.print(pitch);
  Serial.print(" | Roll: ");
  Serial.print(roll);

  Serial.print(" | GyroX: ");
  Serial.print(gyroX);
  Serial.print(" | GyroY: ");
  Serial.print(gyroY);
  Serial.print(" | GyroZ: ");
  Serial.println(gyroZ);

  delay(10); // ~100Hz
}