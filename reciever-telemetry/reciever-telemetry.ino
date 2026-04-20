#include <Wire.h>
#include <MPU6050.h>
#include <SPI.h>
#include <RF24.h>

//LED
#define ARM_LED 2

//LOOP TIMING
#define LOOP_US 4000
unsigned long loopTimer = 0;

// ESC PIN MASKS
#define FL_MASK (1 << 3)  // D3 -> PD3
#define FR_MASK (1 << 5)  // D5 -> PD5
#define BL_MASK (1 << 6)  // D6 -> PD6
#define BR_MASK (1 << 1)  // D9 -> PB1

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

float pitchOffset = 0;
float rollOffset = 0;

float dt;
float alpha = 0.92;

//Filters
static float gyroX_f = 0, gyroY_f = 0, gyroZ_f = 0;
float gyroAlpha = 0.65;

static float accX_f = 0, accY_f = 0, accZ_f = 0;
float accAlpha = 0.65;

//Motors
int m1, m2, m3, m4;
int m1v, m2v, m3v, m4v;

//Armed throttle
int armedThrottle = 1100;

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
DataPacket lastValidData = {512,512,512,512};

unsigned long lastReceiveTime = 0;
const unsigned long FAILSAFE_TIMEOUT = 100;
unsigned long lastTelemetryTime = 0;

//PID
float Kp = 0.6;
float Ki = 0.005;
float Kd = 0.09;

float pitchError, rollError;
float pitchPrevError = 0, rollPrevError = 0;
float pitchIntegral = 0, rollIntegral = 0;
float pitchPID, rollPID;

//YAW
float yawRate, targetYawRate, yawPID;
static float yawIntegral = 0;
float KpYaw = 1.3;
float KiYaw = 0.008;

//ARMING
bool armed = false;

//check if data packet is valid
bool isValid(DataPacket d) {
  if (d.throttle < 0 || d.throttle > 1023) return false;
  if (d.yaw < 0 || d.yaw > 1023) return false;
  if (d.pitch < 0 || d.pitch > 1023) return false;
  if (d.roll < 0 || d.roll > 1023) return false;
  if (d.throttle==0 && d.yaw==0 && d.pitch==0 && d.roll==0) return false;
  return true;
}

void setup() {
  pinMode(ARM_LED, OUTPUT);
  digitalWrite(ARM_LED, LOW);

  Wire.begin();
  Wire.setClock(400000);
  imu.initialize();

  if (!imu.testConnection()) while(1);

  imu.setDLPFMode(MPU6050_DLPF_BW_42);

  // Set motor pins as output
  DDRD |= FL_MASK | FR_MASK | BL_MASK;
  DDRB |= BR_MASK;

  // ESC init pulses
  for (int i = 0; i < 500; i++) {
    PORTD |= FL_MASK | FR_MASK | BL_MASK;
    PORTB |= BR_MASK;

    delayMicroseconds(1000);

    PORTD &= ~(FL_MASK | FR_MASK | BL_MASK);
    PORTB &= ~BR_MASK;

    delayMicroseconds(3000);
  }

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
  int samples = 1000;

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

  loopTimer = micros();
}

void loop() {

  while (micros() - loopTimer < LOOP_US);
  loopTimer = micros();

  dt = LOOP_US / 1000000.0;

  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  accX=ax/16384.0;
  accY=ay/16384.0;
  accZ=az/16384.0;
  
  gyroX=gx/131.0;
  gyroY=gy/131.0;
  gyroZ=gz/131.0;

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

  pitchAcc = atan2(-accX,accZ) * 180/PI;
  rollAcc = atan2(-accY,accZ) * 180/PI;

  if(abs(pitchAcc)>45||abs(rollAcc)>45){
    pitchAcc = pitch;
    rollAcc = roll;
  }

  pitch = alpha * (pitch + gyroY * dt) + (1 - alpha) * pitchAcc;
  roll  = alpha * (roll + gyroX * dt) + (1 - alpha) * rollAcc;

  float pitchFinal = pitch - pitchOffset;
  float rollFinal  = roll - rollOffset;

  if (radio.available()) {
    radio.read(&data, sizeof(data));
    if (isValid(data)) {
      lastValidData = data;
      lastReceiveTime = millis();
    }
  }

  if (millis() - lastReceiveTime > FAILSAFE_TIMEOUT){
    lastValidData={0, 512, 512, 512};
  }

  static unsigned long armStartTime=0;

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

  digitalWrite(ARM_LED, armed);

  int throttle = map(lastValidData.throttle,0,1023,1000,2000);
  if(armed && throttle < armedThrottle) throttle = armedThrottle;

  int targetPitch = map(lastValidData.pitch, 0, 1023, -30, 30);
  int targetRoll  = -map(lastValidData.roll , 0, 1023, -30, 30);
  targetYawRate = map(lastValidData.yaw, 0, 1023, -150, 150);

  if (abs(targetRoll) < 2) targetRoll = 0;
  if (abs(targetPitch) < 2) targetPitch = 0;

  pitchError = targetPitch - pitchFinal;
  rollError  = targetRoll  - rollFinal;

  if (abs(pitchError) < 1) pitchError = 0;
  if (abs(rollError) < 1) rollError = 0;

  pitchIntegral += pitchError*dt;
  rollIntegral  += rollError*dt;

  pitchIntegral = constrain(pitchIntegral, -200, 200);
  rollIntegral = constrain(rollIntegral, -200, 200);

  float pitchDerivative = (pitchError - pitchPrevError)/dt;
  float rollDerivative = (rollError - rollPrevError)/dt;

  pitchDerivative = constrain(pitchDerivative, -250, 250);
  rollDerivative  = constrain(rollDerivative,  -250, 250);

  static float dpf = 0;
  static float rdf = 0;
  dpf = 0.8 * dpf + 0.2 * pitchDerivative;
  rdf = 0.8 * rdf + 0.2 * rollDerivative;

  pitchPID = Kp * pitchError + Ki * pitchIntegral + Kd * dpf;
  rollPID = Kp * rollError + Ki * rollIntegral + Kd * rdf;

  pitchPrevError = pitchError;
  rollPrevError = rollError;

  yawRate = gyroZ;
  float yawError = targetYawRate - yawRate;
  if (abs(yawError)<5) yawError = 0;

  yawIntegral += yawError * dt;
  yawIntegral = constrain(yawIntegral, -50, 50);

  yawPID = KpYaw * yawError + KiYaw * yawIntegral;

  if(throttle <= armedThrottle){
    pitchPID = rollPID = yawPID = 0;
    pitchIntegral = rollIntegral = yawIntegral = 0;
  }

  pitchPID = constrain(pitchPID, -150, 150);
  rollPID = constrain(rollPID , -150, 150);
  yawPID = constrain(yawPID , -150, 150);

  m1v = throttle - pitchPID - rollPID + yawPID;
  m2v = throttle - pitchPID + rollPID - yawPID;
  m3v = throttle + pitchPID - rollPID - yawPID;
  m4v = throttle + pitchPID + rollPID + yawPID;

  m1 = constrain(m1v, 1000, 2000);
  m2 = constrain(m2v, 1000, 2000);
  m3 = constrain(m3v, 1000, 2000);
  m4 = constrain(m4v, 1000, 2000);

  if (!armed) m1 = m2 = m3 = m4 = 1000;

  // Generating port PWM signals
  unsigned long start = micros();

  PORTD |= FL_MASK | FR_MASK | BL_MASK;
  PORTB |= BR_MASK;

  unsigned long t1 = start + m1;
  unsigned long t2 = start + m2;
  unsigned long t3 = start + m3;
  unsigned long t4 = start + m4;

  bool fl = 1,fr = 1,bl = 1,br = 1;

  while(fl||fr||bl||br){
    unsigned long now=micros();

    if(fl && now >= t1){PORTD&=~FL_MASK; fl = 0; }
    if(fr && now >= t2){PORTD&=~FR_MASK; fr = 0; }
    if(bl && now >= t3){PORTD&=~BL_MASK; bl = 0; }
    if(br && now >= t4){PORTB&=~BR_MASK; br = 0; }
  }

  batV = analogRead(A0) * (4.1/1023.0) * 3;
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