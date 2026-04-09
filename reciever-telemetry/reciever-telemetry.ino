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
const byte txAddress[6] = "00001";  // controller → drone
const byte rxAddress[6] = "00002";  // drone → controller

struct DataPacket {
  int16_t throttle;
  int16_t yaw;
  int16_t pitch;
  int16_t roll;
};

struct TelemetryPacket {
  bool armed;
  int throttle;
  float pitch;
  float roll;
  float yawRate;
  int m1, m2, m3, m4;
};

TelemetryPacket telemetry;

DataPacket data;
DataPacket lastValidData = {512, 512, 512, 512};

unsigned long lastReceiveTime = 0;
const unsigned long FAILSAFE_TIMEOUT = 100;

// ================= TELEMETRY TIMER =================
unsigned long lastTelemetryTime = 0;

// ================= MOTORS =================
Servo FL, FR, BL, BR;

// ================= PID =================
float Kp = 1.1;
float Ki = 0;
float Kd = 0.25;

float pitchError, rollError;
float pitchPrevError = 0, rollPrevError = 0;
float pitchIntegral = 0, rollIntegral = 0;

float pitchPID, rollPID;

// ================= YAW =================
float yawRate, targetYawRate, yawPID;
float KpYaw = 1.0;

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
  pinMode(ARM_LED, OUTPUT);
  digitalWrite(ARM_LED, LOW);

  Wire.begin();
  imu.initialize();

  if (!imu.testConnection()) {
    while (1);
  }

  prevTime = millis();

  FL.attach(3);
  FR.attach(5);
  BL.attach(6);
  BR.attach(9);

  FL.writeMicroseconds(1000);
  FR.writeMicroseconds(1000);
  BL.writeMicroseconds(1000);
  BR.writeMicroseconds(1000);
  delay(3000);

  radio.begin();
  radio.openReadingPipe(0, txAddress);   // receive controls
  radio.openWritingPipe(rxAddress);      // send telemetry
  radio.setChannel(108);
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.startListening();
}

// ================= LOOP =================
void loop() {

  // ===== TIME =====
  unsigned long currentTime = millis();
  dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;
  if (dt <= 0) dt = 0.001;
  dt = constrain(dt, 0.001, 0.02);

  // ===== MPU =====
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

  if (!calibrated) {
    pitchOffset = pitch;
    rollOffset  = roll;
    calibrated  = true;
  }

  float pitchFinal = pitch - pitchOffset;
  float rollFinal  = roll  - rollOffset;

  // ===== NRF RECEIVE =====
  if (radio.available()) {
    radio.read(&data, sizeof(data));

    if (isValid(data)) {
      lastValidData = data;
      lastReceiveTime = millis();
    }
  }

  if (millis() - lastReceiveTime > FAILSAFE_TIMEOUT) {
    lastValidData = {0, 512, 512, 512};
  }

  // ===== ARMING =====
  static unsigned long armStartTime = 0;

  if (lastValidData.throttle < 50 && lastValidData.yaw > 900) {
    if (armStartTime == 0) armStartTime = millis();
    if (millis() - armStartTime > 1000) armed = true;
  } else {
    armStartTime = 0;
  }

  if (lastValidData.throttle < 50 && lastValidData.yaw < 100) {
    armed = false;
  }

  digitalWrite(ARM_LED, armed ? HIGH : LOW);

  // ===== INPUT =====
  int throttle = map(lastValidData.throttle, 0, 1023, 1000, 2000);
  if (armed && throttle < 1050) throttle = 1050;

  float targetPitch = map(lastValidData.pitch, 0, 1023, -30, 30);
  float targetRoll  = map(lastValidData.roll,  0, 1023, -30, 30);
  targetYawRate     = map(lastValidData.yaw,   0, 1023, -150, 150);

  // ===== PID =====
  pitchError = targetPitch - pitchFinal;
  rollError  = targetRoll  - rollFinal;
  
  if (abs(pitchError) < 1) pitchError = 0;
  if (abs(rollError)  < 1) rollError  = 0;

  pitchIntegral += pitchError * dt;
  rollIntegral  += rollError  * dt;

  pitchIntegral = constrain(pitchIntegral, -200, 200);
  rollIntegral  = constrain(rollIntegral,  -200, 200);

  float pitchDerivative = (pitchError - pitchPrevError) / dt;
  float rollDerivative  = (rollError  - rollPrevError)  / dt;

  static float pitchDerivativeFiltered = 0;
  static float rollDerivativeFiltered = 0;

  pitchDerivativeFiltered = 0.8 * pitchDerivativeFiltered + 0.2 * pitchDerivative;
  rollDerivativeFiltered  = 0.8 * rollDerivativeFiltered  + 0.2 * rollDerivative;

  pitchPID = Kp * pitchError + Ki * pitchIntegral + Kd * pitchDerivativeFiltered;
  rollPID  = Kp * rollError  + Ki * rollIntegral  + Kd * rollDerivativeFiltered;
  
  pitchPrevError = pitchError;
  rollPrevError  = rollError;

  // ===== YAW =====
  yawRate = gyroZ;
  float yawError = targetYawRate - yawRate;
  yawPID = KpYaw * yawError;

  if (throttle <= 1050) {
    pitchPID = rollPID = yawPID = 0;
    pitchIntegral = rollIntegral = 0;
  }
  
  pitchPID = constrain(pitchPID, -200, 200);
  rollPID  = constrain(rollPID,  -200, 200);
  yawPID   = constrain(yawPID,   -150, 150);

  // ===== MIXING =====
  int m1 = constrain(throttle - pitchPID - rollPID + yawPID, 1000, 2000);
  int m2 = constrain(throttle - pitchPID + rollPID - yawPID, 1000, 2000);
  int m3 = constrain(throttle + pitchPID - rollPID - yawPID + 30, 1000, 2000); // different esc
  int m4 = constrain(throttle + pitchPID + rollPID + yawPID, 1000, 2000);

  if (!armed) m1 = m2 = m3 = m4 = 1000;

  FL.writeMicroseconds(m1);
  FR.writeMicroseconds(m2);
  BL.writeMicroseconds(m3);
  BR.writeMicroseconds(m4);

  // ===== TELEMETRY =====
  if (millis() - lastTelemetryTime > 20) {

    telemetry.armed = armed;
    telemetry.throttle = throttle;
    telemetry.pitch = pitchFinal;
    telemetry.roll = rollFinal;
    telemetry.yawRate = yawRate;
    telemetry.m1 = m1;
    telemetry.m2 = m2;
    telemetry.m3 = m3;
    telemetry.m4 = m4;

    radio.writeAckPayload(0, &telemetry, sizeof(telemetry));

    lastTelemetryTime = millis();
  }
}