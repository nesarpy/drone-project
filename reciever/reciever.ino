#include <Wire.h>
#include <MPU6050.h>
#include <SPI.h>
#include <RF24.h>
#include <Servo.h>

// ================= LED =================
#define ARM_LED 2

// ================= MPU =================
MPU6050 imu;

int16_t ax, ay, az, gx, gy, gz;

float accX, accY, accZ;
float gyroX, gyroY, gyroZ;

float pitch = 0, roll = 0;
float pitchAcc, rollAcc;

// Calibration offsets
float pitchOffset = 0;
float rollOffset = 0;
bool calibrated = false;

unsigned long prevTime;
float dt;
float alpha = 0.98;

// ================= NRF =================
RF24 radio(7, 8);
const byte address[6] = "00001";

struct DataPacket {
  int16_t throttle;
  int16_t yaw;
  int16_t pitch;
  int16_t roll;
};

DataPacket data;
DataPacket lastValidData = {512, 512, 512, 512};

unsigned long lastReceiveTime = 0;
const unsigned long FAILSAFE_TIMEOUT = 100;

// ================= MOTORS =================
Servo FL, FR, BL, BR;

// ================= PID =================
float Kp = 1.7;
float Ki = 0;
float Kd = 0.6;

float pitchError, rollError;
float pitchPrevError = 0, rollPrevError = 0;
float pitchIntegral = 0, rollIntegral = 0;

float pitchPID, rollPID;

// ================= YAW =================
float yawRate, targetYawRate, yawPID;
float KpYaw = 2.0;

// ================= ARMING =================
bool armed = false;

// ================= FUNCTIONS =================
bool isValid(DataPacket d) {
  if (d.throttle < 0 || d.throttle > 1023) return false;
  if (d.yaw < 0 || d.yaw > 1023) return false;
  if (d.pitch < 0 || d.pitch > 1023) return false;
  if (d.roll < 0 || d.roll > 1023) return false;

  if (d.throttle == 0 && d.yaw == 0 &&
      d.pitch == 0 && d.roll == 0) return false;

  return true;
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  pinMode(ARM_LED, OUTPUT);
  digitalWrite(ARM_LED, LOW);

  Wire.begin();
  imu.initialize();

  if (!imu.testConnection()) {
    Serial.println("MPU FAILED");
    while (1);
  }

  prevTime = millis();

  FL.attach(3);
  FR.attach(5);
  BL.attach(6);
  BR.attach(9);

  // ESC initialization
  FL.writeMicroseconds(1000);
  FR.writeMicroseconds(1000);
  BL.writeMicroseconds(1000);
  BR.writeMicroseconds(1000);
  delay(3000);

  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setChannel(108);
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
  radio.startListening();

  Serial.println("FLIGHT CONTROLLER READY");
}

// ================= LOOP =================
void loop() {

  // ===== TIME =====
  unsigned long currentTime = millis();
  dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  if (dt <= 0) dt = 0.001;

  // ===== MPU =====
  // MPU is mounted 90deg rotated, so X and Y axes are physically swapped
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  accX = ax / 16384.0;
  accY = ay / 16384.0;
  accZ = az / 16384.0;

  gyroX = gx / 131.0;
  gyroY = gy / 131.0;
  gyroZ = gz / 131.0;

  pitchAcc = atan2(-accX, accZ) * 180 / PI;
  rollAcc  = atan2(accY, accZ) * 180 / PI;

  pitch = alpha * (pitch + gyroY * dt) + (1 - alpha) * pitchAcc;
  roll  = alpha * (roll  + gyroX * dt) + (1 - alpha) * rollAcc;

  // ===== CALIBRATION =====
  if (!calibrated) {
    pitchOffset = pitch;
    rollOffset  = roll;
    calibrated  = true;
  }

  float pitchFinal = pitch - pitchOffset;
  float rollFinal  = roll  - rollOffset;

  // ===== NRF =====
  if (radio.available()) {
    radio.read(&data, sizeof(data));

    if (isValid(data)) {
      lastValidData = data;
      lastReceiveTime = millis();
    }
  }

  if (millis() - lastReceiveTime > FAILSAFE_TIMEOUT) {
    lastValidData.throttle = 0;
    lastValidData.yaw = 512;
    lastValidData.pitch = 512;
    lastValidData.roll = 512;
  }

  // ===== ARMING =====
  static unsigned long armStartTime = 0;

  if (lastValidData.throttle < 50 && lastValidData.yaw > 900) {
    if (armStartTime == 0) armStartTime = millis();

    if (millis() - armStartTime > 1000) {
      armed = true;
    }
  } else {
    armStartTime = 0;
  }

  if (lastValidData.throttle < 50 && lastValidData.yaw < 100) {
    armed = false;
  }

  // ===== LED =====
  digitalWrite(ARM_LED, armed ? HIGH : LOW);

  // ===== INPUT =====
  int throttle = map(lastValidData.throttle, 0, 1023, 1000, 2000);

  if (armed && throttle < 1050) throttle = 1050;

  float targetPitch = map(lastValidData.pitch, 0, 1023, -30, 30);
  float targetRoll = map(lastValidData.roll,  0, 1023, -30, 30);
  targetYawRate = map(lastValidData.yaw,   0, 1023, -150, 150);

  // ===== PID =====
  pitchError = targetPitch - pitchFinal;
  rollError  = targetRoll  - rollFinal;

  pitchIntegral += pitchError * dt;
  rollIntegral  += rollError  * dt;

  // Clamp integrals to prevent windup
  pitchIntegral = constrain(pitchIntegral, -200, 200);
  rollIntegral  = constrain(rollIntegral,  -200, 200);

  float pitchDerivative = (pitchError - pitchPrevError) / dt;
  float rollDerivative  = (rollError  - rollPrevError)  / dt;

  pitchPID = Kp * pitchError + Ki * pitchIntegral + Kd * pitchDerivative;
  rollPID  = Kp * rollError  + Ki * rollIntegral  + Kd * rollDerivative;

  pitchPrevError = pitchError;
  rollPrevError  = rollError;

  // ===== YAW =====
  yawRate = gyroZ;
  float yawError = targetYawRate - yawRate;
  yawPID = KpYaw * yawError;

  // ===== SAFETY =====
  if (throttle <= 1050) {
    pitchPID     = 0;
    rollPID      = 0;
    yawPID       = 0;
    pitchIntegral = 0;
    rollIntegral  = 0;
  }

  // ===== MIXING =====
  int m1 = throttle - pitchPID - rollPID + yawPID;
  int m2 = throttle - pitchPID + rollPID - yawPID;
  int m3 = throttle + pitchPID - rollPID - yawPID;
  int m4 = throttle + pitchPID + rollPID + yawPID;

  m1 = constrain(m1, 1000, 2000);
  m2 = constrain(m2, 1000, 2000);
  m3 = constrain(m3, 1000, 2000);
  m4 = constrain(m4, 1000, 2000);

  if (!armed) {
    m1 = m2 = m3 = m4 = 1000;
  }

  // ===== OUTPUT =====
  FL.writeMicroseconds(m1);
  FR.writeMicroseconds(m2);
  BL.writeMicroseconds(m3);
  BR.writeMicroseconds(m4);

  // ===== DEBUG =====
  Serial.print("ARMED: ");    Serial.print(armed);
  Serial.print(" | Throttle: "); Serial.print(throttle);
  Serial.print(" | Pitch: ");    Serial.print(pitchFinal);
  Serial.print(" | Roll: ");     Serial.print(rollFinal);
  Serial.print(" | YawRate: ");  Serial.print(yawRate);
  Serial.print(" | PID P: ");    Serial.print(pitchPID);
  Serial.print(" | PID R: ");    Serial.print(rollPID);
  Serial.print(" | Motors: ");
  Serial.print(m1); Serial.print(" ");
  Serial.print(m2); Serial.print(" ");
  Serial.print(m3); Serial.print(" ");
  Serial.print(m4);
  Serial.println();
}