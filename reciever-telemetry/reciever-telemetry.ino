#include <Wire.h>
#include <MPU6050.h>
#include <SPI.h>
#include <RF24.h>
#include <Servo.h>

// ─────────────────────────────────────────────
// PINS
// ─────────────────────────────────────────────
#define ARM_LED 2

#define FL_PIN 3
#define FR_PIN 5
#define BL_PIN 6
#define BR_PIN 4

#define BATTERY_PIN A0

// ─────────────────────────────────────────────
// LOOP
// ─────────────────────────────────────────────
#define LOOP_US 5000

// ─────────────────────────────────────────────
// RADIO
// ─────────────────────────────────────────────
RF24 radio(7, 8);
const byte txAddress[6] = "00001";
const byte rxAddress[6] = "00002";

struct ControlPacket {
  int16_t throttle, yaw, pitch, roll;
};

struct TelemetryPacket {
  bool armed;
  int throttle;
  float pitch;
  float roll;
  float yawRate;
  int m1, m2, m3, m4;
  float batV;
};

ControlPacket data;
ControlPacket lastValidData = {512,512,512,512};
TelemetryPacket telemetry;

unsigned long lastReceiveTime = 0;
unsigned long lastTelemetryTime = 0;
const unsigned long FAILSAFE_TIMEOUT = 100;

// ─────────────────────────────────────────────
// IMU
// ─────────────────────────────────────────────
MPU6050 imu;

int16_t ax, ay, az, gx, gy, gz;

float gyroX, gyroY, gyroZ;
float accX, accY, accZ;

float gyroX_f = 0, gyroY_f = 0, gyroZ_f = 0;
float accX_f = 0, accY_f = 0, accZ_f = 0;

float gyroAlpha = 0.6;
float accAlpha  = 0.65;

float pitch = 0, roll = 0;
float pitchAcc, rollAcc;

float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;

float pitchOffset = 0, rollOffset = 0;
float alpha = 0.93;

// ─────────────────────────────────────────────
// PID
// ─────────────────────────────────────────────
float Kp = 0.8;
float Ki = 0.001;
float Kd = 0.1;

float rollSetpoint, pitchSetpoint, yawSetpoint;

float rollError, pitchError;
float rollIntegral = 0, pitchIntegral = 0;
float rollLastError = 0, pitchLastError = 0;

float rollPID, pitchPID;
float rollDerivative;

float yawIntegral = 0, yawPID;
float KpYaw = 0.5;
float KiYaw = 0.001;

// ─────────────────────────────────────────────
// MOTORS
// ─────────────────────────────────────────────
int m1, m2, m3, m4;
int m1v, m2v, m3v, m4v;

Servo m1_servo, m2_servo, m3_servo, m4_servo;

bool armed = false;
const int MOTOR_IDLE = 1100;

// ─────────────────────────────────────────────
// BATTERY
// ─────────────────────────────────────────────
float batV = 0;
float batV_filtered = 0;

// ─────────────────────────────────────────────
// VALIDATION
// ─────────────────────────────────────────────
bool isValid(ControlPacket d) {
  if (d.throttle < 0 || d.throttle > 1023) return false;
  if (d.yaw < 0 || d.yaw > 1023) return false;
  if (d.pitch < 0 || d.pitch > 1023) return false;
  if (d.roll < 0 || d.roll > 1023) return false;
  if (d.throttle==0 && d.yaw==0 && d.pitch==0 && d.roll==0) return false;
  return true;
}

// ─────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────
void setup() {

  pinMode(ARM_LED, OUTPUT);

  Wire.begin();
  Wire.setClock(400000);
  imu.initialize();
  imu.setDLPFMode(MPU6050_DLPF_BW_42);

  // Attach ESCs
  m1_servo.attach(FL_PIN);
  m2_servo.attach(FR_PIN);
  m3_servo.attach(BL_PIN);
  m4_servo.attach(BR_PIN);

  for (int i = 0; i < 200; i++) {
    m1_servo.writeMicroseconds(1000);
    m2_servo.writeMicroseconds(1000);
    m3_servo.writeMicroseconds(1000);
    m4_servo.writeMicroseconds(1000);
    delay(10);
  }
  delay(500);
  
  // RADIO (EXACT SAME)
  radio.begin();
  radio.setAutoAck(true);
  radio.openReadingPipe(0, txAddress);
  radio.openWritingPipe(rxAddress);
  radio.setChannel(108);
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.startListening();

  // CALIBRATION
  float ps = 0, rs = 0;
  float gxs = 0, gys = 0, gzs = 0;

  for (int i = 0; i < 1000; i++) {
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    accX = ax / 16384.0;
    accY = ay / 16384.0;
    accZ = az / 16384.0;

    ps += atan2(-accX, accZ) * 180 / PI;
    rs += atan2(-accY, accZ) * 180 / PI;

    gxs += gx;
    gys += gy;
    gzs += gz;

    delay(5);
  }

  pitchOffset = ps / 1000;
  rollOffset  = rs / 1000;

  gyroX_offset = (gxs / 1000.0) / 131.0;
  gyroY_offset = (gys / 1000.0) / 131.0;
  gyroZ_offset = (gzs / 1000.0) / 131.0;
}

// ─────────────────────────────────────────────
// LOOP
// ─────────────────────────────────────────────
void loop() {

  static unsigned long lastLoopTime = 0;

  unsigned long now = micros();

  if (now - lastLoopTime < LOOP_US) return;

  float dt = (now - lastLoopTime) / 1000000.0;
  lastLoopTime = now;

  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  gyroX = gx / 131.0 - gyroX_offset;
  gyroY = gy / 131.0 - gyroY_offset;
  gyroZ = gz / 131.0 - gyroZ_offset;

  accX = ax / 16384.0;
  accY = ay / 16384.0;
  accZ = az / 16384.0;

  pitchAcc = atan2(-accX, accZ) * 180 / PI;
  rollAcc  = atan2(-accY, accZ) * 180 / PI;

  pitch = alpha * (pitch + gyroY * dt) + (1 - alpha) * pitchAcc;
  roll  = alpha * (roll  + gyroX * dt) + (1 - alpha) * rollAcc;

  float pitchFinal = pitch - pitchOffset;
  float rollFinal  = roll  - rollOffset;

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

  static unsigned long armStartTime = 0;

  if (lastValidData.throttle < 50 && lastValidData.yaw > 900) {
    if (armStartTime == 0) armStartTime = millis();
    if (millis() - armStartTime > 1000) armed = true;
  } else {
    armStartTime = 0;
  }

  if (lastValidData.throttle < 50 && lastValidData.yaw < 100) {
    armed = false;
    rollIntegral = pitchIntegral = yawIntegral = 0;
  }

  digitalWrite(ARM_LED, armed);

  int throttle = map(lastValidData.throttle, 0, 1023, 1000, 2000);

  int rollInput  = -(lastValidData.roll - 512);
  int pitchInput = lastValidData.pitch - 512;
  int yawInput   = lastValidData.yaw - 512;

  if (abs(rollInput) < 20) rollInput = 0;
  if (abs(pitchInput) < 20) pitchInput = 0;
  if (abs(yawInput) < 20) yawInput = 0;

  float angleKp = 3.0;

  float rollAngleSetpoint  = rollInput * 0.1;
  float pitchAngleSetpoint = pitchInput * 0.1;

  float rollAngleError  = rollAngleSetpoint  - rollFinal;
  float pitchAngleError = pitchAngleSetpoint - pitchFinal;

  rollSetpoint  = angleKp * rollAngleError;
  pitchSetpoint = angleKp * pitchAngleError;
  yawSetpoint = yawInput * 0.5;

  rollError  = gyroX - rollSetpoint;
  pitchError = gyroY - pitchSetpoint;

  rollIntegral  = constrain(rollIntegral  + rollError * dt,  -200, 200);
  pitchIntegral = constrain(pitchIntegral + pitchError * dt, -200, 200);

  rollDerivative = (rollError - rollLastError) / dt;
  rollDerivative = constrain(rollDerivative, -200, 200);

  rollPID  = -(Kp * rollError  + Ki * rollIntegral  + Kd * rollDerivative);
  pitchPID = -(Kp * pitchError + Ki * pitchIntegral + Kd * (pitchError - pitchLastError) / dt);

  rollLastError  = rollError;
  pitchLastError = pitchError;

  float yawError = gyroZ - yawSetpoint;
  yawIntegral = constrain(yawIntegral + yawError * dt, -150, 150);
  yawPID = KpYaw * yawError + KiYaw * yawIntegral;

  if (!armed) {
    m1 = m2 = m3 = m4 = 1000;
  } 
  else if (throttle < MOTOR_IDLE) {
    m1 = m2 = m3 = m4 = MOTOR_IDLE;
  } 
  else {
    m1v = throttle - pitchPID - rollPID + yawPID;
    m2v = throttle - pitchPID + rollPID - yawPID;
    m3v = throttle + pitchPID - rollPID - yawPID;
    m4v = throttle + pitchPID + rollPID + yawPID;

    m1 = constrain(m1v, 1000, 2000);
    m2 = constrain(m2v*0.99, 1000, 2000);
    m3 = constrain(m3v*1.012, 1000, 2000);
    m4 = constrain(m4v, 1000, 2000);
  }

  m1_servo.writeMicroseconds(m1);
  m2_servo.writeMicroseconds(m2);
  m3_servo.writeMicroseconds(m3);
  m4_servo.writeMicroseconds(m4);

  batV = analogRead(A0) * (5.1 / 1023.0) * 3;
  batV_filtered = 0.9 * batV_filtered + 0.1 * batV;
  batV = batV_filtered;

  if (millis() - lastTelemetryTime > 20) {
    telemetry.armed = armed;
    telemetry.throttle = throttle;
    telemetry.pitch = pitchFinal;
    telemetry.roll = rollFinal;
    telemetry.yawRate = gyroZ;
    telemetry.m1 = m1;
    telemetry.m2 = m2;
    telemetry.m3 = m3;
    telemetry.m4 = m4;
    telemetry.batV = batV;

    radio.writeAckPayload(0, &telemetry, sizeof(telemetry));
    lastTelemetryTime = millis();
  }
}