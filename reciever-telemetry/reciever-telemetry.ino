#include <Wire.h>
#include <MPU6050.h>
#include <SPI.h>
#include <RF24.h>

// ─────────────────────────────────────────────
// PINS
// ─────────────────────────────────────────────
#define ARM_LED 2

#define FL_MASK (1 << 3)
#define FR_MASK (1 << 5)
#define BL_MASK (1 << 6)
#define BR_MASK (1 << 1)

#define BATTERY_PIN A0

// ─────────────────────────────────────────────
// LOOP
// ─────────────────────────────────────────────
#define LOOP_US 4000

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
float Kp = 0.75;
float Ki = 0;
float Kd = 0.08;

float rollSetpoint, pitchSetpoint, yawSetpoint;

float rollError, pitchError;
float rollIntegral = 0, pitchIntegral = 0;
float rollLastError = 0, pitchLastError = 0;

float rollPID, pitchPID;

float yawIntegral = 0, yawPID;
float KpYaw = 1.2;
float KiYaw = 0.01;

// ─────────────────────────────────────────────
// MOTORS
// ─────────────────────────────────────────────
int m1, m2, m3, m4;
int m1v, m2v, m3v, m4v;

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

  DDRD |= FL_MASK | FR_MASK | BL_MASK;
  DDRB |= BR_MASK;

  // ESC warmup
  for (int i = 0; i < 500; i++) {
    PORTD |= FL_MASK | FR_MASK | BL_MASK;
    PORTB |= BR_MASK;
    delayMicroseconds(1000);
    PORTD &= ~(FL_MASK | FR_MASK | BL_MASK);
    PORTB &= ~BR_MASK;
    delayMicroseconds(3000);
  }

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

  // gyro offsets (convert to deg/s like runtime)
  gyroX_offset = (gxs / 1000.0) / 131.0;
  gyroY_offset = (gys / 1000.0) / 131.0;
  gyroZ_offset = (gzs / 1000.0) / 131.0;
}

// ─────────────────────────────────────────────
// LOOP
// ─────────────────────────────────────────────
void loop() {

  unsigned long start = micros();

  // ===== IMU =====
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  gyroX = gx / 131.0 - gyroX_offset;
  gyroY = gy / 131.0 - gyroY_offset;
  gyroZ = gz / 131.0 - gyroZ_offset;

  accX = ax / 16384.0;
  accY = ay / 16384.0;
  accZ = az / 16384.0;

  gyroX_f = gyroAlpha * gyroX_f + (1 - gyroAlpha) * gyroX;
  gyroY_f = gyroAlpha * gyroY_f + (1 - gyroAlpha) * gyroY;
  gyroZ_f = gyroAlpha * gyroZ_f + (1 - gyroAlpha) * gyroZ;

  accX_f = accAlpha * accX_f + (1 - accAlpha) * accX;
  accY_f = accAlpha * accY_f + (1 - accAlpha) * accY;
  accZ_f = accAlpha * accZ_f + (1 - accAlpha) * accZ;

  gyroX = gyroX_f;
  gyroY = gyroY_f;
  gyroZ = gyroZ_f;

  accX = accX_f;
  accY = accY_f;
  accZ = accZ_f;

  pitchAcc = atan2(-accX, accZ) * 180 / PI;
  rollAcc  = atan2(-accY, accZ) * 180 / PI;

  pitch = alpha * (pitch + gyroY * 0.004) + (1 - alpha) * pitchAcc;
  roll  = alpha * (roll  + gyroX * 0.004) + (1 - alpha) * rollAcc;

  float pitchFinal = pitch - pitchOffset;
  float rollFinal  = roll  - rollOffset;

  // ===== RADIO (UNCHANGED) =====
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
    rollIntegral = pitchIntegral = yawIntegral = 0;
  }

  digitalWrite(ARM_LED, armed);

  // INPUT
  int throttle = map(lastValidData.throttle, 0, 1023, 1000, 2000);

  int rollInput  = -(lastValidData.roll - 512);
  int pitchInput = lastValidData.pitch - 512;
  int yawInput   = lastValidData.yaw - 512;

  if (abs(rollInput) < 20) rollInput = 0;
  if (abs(pitchInput) < 20) pitchInput = 0;
  if (abs(yawInput) < 20) yawInput = 0;

  rollSetpoint  = rollInput * 0.4;
  pitchSetpoint = pitchInput * 0.4;
  yawSetpoint   = yawInput * 0.5;

  rollSetpoint  -= rollFinal * 2.0;
  pitchSetpoint -= pitchFinal * 2.0;

  // PID
  rollError  = gyroX - rollSetpoint;
  pitchError = gyroY - pitchSetpoint;

  rollIntegral  = constrain(rollIntegral  + rollError,  -200, 200);
  pitchIntegral = constrain(pitchIntegral + pitchError, -200, 200);

  rollPID  = -(Kp * rollError  + Ki * rollIntegral  + Kd * (rollError  - rollLastError));
  pitchPID = -(Kp * pitchError + Ki * pitchIntegral + Kd * (pitchError - pitchLastError));

  rollLastError  = rollError;
  pitchLastError = pitchError;

  float yawError = gyroZ - yawSetpoint;
  yawIntegral = constrain(yawIntegral + yawError, -150, 150);
  yawPID = KpYaw * yawError + KiYaw * yawIntegral;

  // MOTOR LOGIC
  if (!armed) {
    m1 = m2 = m3 = m4 = 1000;
  } 
  else if (throttle < 1100) {
    m1 = m2 = m3 = m4 = MOTOR_IDLE;
  } 
  else {
    m1v = throttle - pitchPID - rollPID + yawPID;
    m2v = throttle - pitchPID + rollPID - yawPID;
    m3v = throttle + pitchPID - rollPID - yawPID;
    m4v = throttle + pitchPID + rollPID + yawPID;

    m1 = constrain(m1v, MOTOR_IDLE, 2000);
    m2 = constrain(m2v, MOTOR_IDLE, 2000);
    m3 = constrain(m3v, MOTOR_IDLE, 2000);
    m4 = constrain(m4v, MOTOR_IDLE, 2000);
  }

  // PWM
  unsigned long pwmStart = micros();

  PORTD |= FL_MASK | FR_MASK | BL_MASK;
  PORTB |= BR_MASK;

  unsigned long t1 = pwmStart + m1;
  unsigned long t2 = pwmStart + m2;
  unsigned long t3 = pwmStart + m3;
  unsigned long t4 = pwmStart + m4;

  bool fl = 1, fr = 1, bl = 1, br = 1;

  while (fl || fr || bl || br) {
    unsigned long now = micros();

    if (fl && now >= t1) { PORTD &= ~FL_MASK; fl = 0; }
    if (fr && now >= t2) { PORTD &= ~FR_MASK; fr = 0; }
    if (bl && now >= t3) { PORTD &= ~BL_MASK; bl = 0; }
    if (br && now >= t4) { PORTB &= ~BR_MASK; br = 0; }
  }

  // BATTERY
  batV = analogRead(A0) * (4.1 / 1023.0) * 3;
  batV_filtered = 0.9 * batV_filtered + 0.1 * batV;
  batV = batV_filtered;

  // TELEMETRY (UNCHANGED)
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

  // LOOP TIMING
  while (micros() - start < LOOP_US);
}