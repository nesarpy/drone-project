#include <Wire.h>
#include <MPU6050.h>
#include <SPI.h>
#include <RF24.h>
#include <Servo.h>

//LED
#define ARM_LED 2

//LOOP TIMING
#define LOOP_US 4000  // 250Hz loop rate (4ms)
unsigned long loopTimer = 0;

//BATTERY
float batV = 0;
static float batV_filtered = 0;

//MPU
MPU6050 imu;

int16_t ax, ay, az, gx, gy, gz;

float accX, accY, accZ;
float gyroX, gyroY, gyroZ;

float pitch = 0, roll = 0;
float pitchAcc, rollAcc;

// Calibration offsets
float pitchOffset = 0;
float rollOffset = 0;

float dt;
float alpha = 0.93;

//Gyro LPF
static float gyroX_f = 0, gyroY_f = 0, gyroZ_f = 0;
float gyroAlpha = 0.7;

//Acc LPF
static float accX_f = 0, accY_f = 0, accZ_f = 0;
float accAlpha = 0.65;

//MOTOR MIXING VARS
int m1, m2, m3, m4;
int m1v, m2v, m3v, m4v;

//NRF
RF24 radio(7, 8);
const byte txAddress[6] = "00001";
const byte rxAddress[6] = "00002";

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
  float batV;
};

TelemetryPacket telemetry;

DataPacket data;
DataPacket lastValidData = {512, 512, 512, 512};

unsigned long lastReceiveTime = 0;
const unsigned long FAILSAFE_TIMEOUT = 100;

unsigned long lastTelemetryTime = 0;

Servo FL, FR, BL, BR;
float trim = 0;

//PID
float Kp = 1.3;
float Ki = 0.01;
float Kd = 0.3;

float pitchError, rollError;
float pitchPrevError = 0, rollPrevError = 0;
float pitchIntegral = 0, rollIntegral = 0;

float pitchPID, rollPID;

//YAW
float yawRate, targetYawRate, yawPID;
static float yawIntegral = 0;
float KpYaw = 1.7;
float KiYaw = 0.005;

//ARMING
bool armed = false;

//FUNCTIONS
bool isValid(DataPacket d) {
  if (d.throttle < 0 || d.throttle > 1023) return false;
  if (d.yaw < 0 || d.yaw > 1023) return false;
  if (d.pitch < 0 || d.pitch > 1023) return false;
  if (d.roll < 0 || d.roll > 1023) return false;
  if (d.throttle == 0 && d.yaw == 0 &&
      d.pitch == 0 && d.roll == 0) return false;
  return true;
}

void setup() {
  pinMode(ARM_LED, OUTPUT);
  digitalWrite(ARM_LED, LOW);

  Wire.begin();
  Wire.setClock(400000);
  imu.initialize();

  if (!imu.testConnection()) {
    while (1);
  }
  
  imu.setDLPFMode(MPU6050_DLPF_BW_42);

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
  radio.setAutoAck(true);
  radio.openReadingPipe(0, txAddress);
  radio.openWritingPipe(rxAddress);
  radio.setChannel(108);
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.startListening();

  float pitchSum = 0, rollSum = 0;
  int samples = 500;

  for (int i = 0; i < samples; i++) {
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    accX = ax / 16384.0;
    accY = ay / 16384.0;
    accZ = az / 16384.0;
    pitchAcc = atan2(-accX, accZ) * 180 / PI;
    rollAcc  = atan2(-accY, accZ) * 180 / PI;
    pitchSum += pitchAcc;
    rollSum  += rollAcc;
    delay(5);
  }

  pitchOffset = pitchSum / samples;
  rollOffset  = rollSum / samples;

  // Start loop timer AFTER all setup is done
  loopTimer = micros();
}

void loop() {

  //LOOP RATE CONTROL
  //Wait here until exactly LOOP_US microseconds have passed since last iteration.
  //This makes dt constant and predictable regardless of code execution time.
  while (micros() - loopTimer < LOOP_US);
  loopTimer = micros();

  dt = LOOP_US / 1000000.0;  // = 0.004 seconds

  //MPU
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  accX = ax / 16384.0;
  accY = ay / 16384.0;
  accZ = az / 16384.0;

  gyroX = gx / 131.0;
  gyroY = gy / 131.0;
  gyroZ = gz / 131.0;

  //Gyro LPF
  gyroX_f = gyroAlpha * gyroX_f + (1 - gyroAlpha) * gyroX;
  gyroY_f = gyroAlpha * gyroY_f + (1 - gyroAlpha) * gyroY;
  gyroZ_f = gyroAlpha * gyroZ_f + (1 - gyroAlpha) * gyroZ;

  //Acc LPF
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

  if (abs(pitchAcc) > 45 || abs(rollAcc) > 45) {
    pitchAcc = pitch;
    rollAcc  = roll;
  }

  pitch = alpha * (pitch + gyroY * dt) + (1 - alpha) * pitchAcc;
  roll  = alpha * (roll  + gyroX * dt) + (1 - alpha) * rollAcc;

  float pitchFinal = pitch - pitchOffset;
  float rollFinal  = roll  - rollOffset;

  //NRF RECEIVE
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

  //ARMING
  static unsigned long armStartTime = 0;

  if (lastValidData.throttle < 50 && lastValidData.yaw > 900) {
    if (armStartTime == 0) armStartTime = millis();
    if (millis() - armStartTime > 1000) armed = true;
  } else {
    armStartTime = 0;
  }

  //reset integrals on disarm
  if (lastValidData.throttle < 50 && lastValidData.yaw < 100) {
    armed = false;
    pitchIntegral = 0;
    rollIntegral = 0;
    yawIntegral = 0;
  }

  digitalWrite(ARM_LED, armed ? HIGH : LOW);

  //INPUT
  int throttle = map(lastValidData.throttle, 0, 1023, 1000, 2000);
  if (armed && throttle < 1070) throttle = 1070;

  float targetPitch = map(lastValidData.pitch, 0, 1023, -30, 30);
  float targetRoll = map(lastValidData.roll, 0, 1023, -30, 30);
  targetYawRate = map(lastValidData.yaw, 0, 1023, -150, 150);

  if (abs(targetRoll) < 2) targetRoll  = 0;
  if (abs(targetPitch) < 2) targetPitch = 0;

  //PID
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

  pitchDerivativeFiltered = 0.9 * pitchDerivativeFiltered + 0.1 * pitchDerivative;
  rollDerivativeFiltered  = 0.9 * rollDerivativeFiltered  + 0.1 * rollDerivative;

  pitchPID = Kp * pitchError + Ki * pitchIntegral + Kd * pitchDerivativeFiltered;
  rollPID  = Kp * rollError  + Ki * rollIntegral  + Kd * rollDerivativeFiltered;

  pitchPrevError = pitchError;
  rollPrevError  = rollError;

  //YAW
  yawRate = gyroZ;
  float yawError = targetYawRate - yawRate;
  if (abs(yawError) < 5) yawError = 0;
  yawIntegral += yawError * dt;
  yawIntegral = constrain(yawIntegral, -50, 50);
  yawPID = KpYaw * yawError + KiYaw * yawIntegral;

  if (throttle <= 1070) {
    pitchPID = rollPID = yawPID = 0;
    pitchIntegral = rollIntegral = 0;
    yawIntegral = 0;
  }

  pitchPID = constrain(pitchPID, -200, 200);
  rollPID = constrain(rollPID, -200, 200);
  yawPID = constrain(yawPID, -150, 150);

  //MIXING
  m1v = throttle - pitchPID - rollPID + yawPID;
  m2v = throttle - pitchPID + rollPID - yawPID;
  m3v = throttle + pitchPID - rollPID - yawPID;
  m4v = throttle + pitchPID + rollPID + yawPID;
  m1 = constrain(m1v, 1000, 2000);
  m2 = constrain(m2v, 1000, 2000);
  m3 = constrain(m3v, 1000, 2000);
  m4 = constrain(m4v, 1000, 2000);

  if (!armed) m1 = m2 = m3 = m4 = 1000;

  FL.writeMicroseconds(m1);
  FR.writeMicroseconds(m2);
  BL.writeMicroseconds(m3);
  BR.writeMicroseconds(m4);

  batV = analogRead(A0) * (4.1 / 1023.0) * 3;
  batV_filtered = 0.9 * batV_filtered + 0.1 * batV;
  batV = batV_filtered;

  //TELEMETRY
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
    telemetry.batV = batV;

    radio.writeAckPayload(0, &telemetry, sizeof(telemetry));
    lastTelemetryTime = millis();
  }
}