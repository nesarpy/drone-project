#include <SPI.h>
#include <RF24.h>

RF24 radio(7, 8);
const byte txAddress[6] = "00001";  // controller to drone
const byte rxAddress[6] = "00002";  // drone to controller

int LjoyX = A0;
int LjoyY = A2;
int RjoyX = A4;
int RjoyY = A6;

int LED = 4;
int BATLED = 9;

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

DataPacket data;
TelemetryPacket telemetry;

void setup() {
  Serial.begin(250000);

  pinMode(LED, OUTPUT);
  pinMode(BATLED, OUTPUT);

  radio.begin();
  radio.setAutoAck(true);
  radio.openWritingPipe(txAddress);      // send controls
  radio.openReadingPipe(1, rxAddress);   // receive telemetry
  radio.setChannel(108);
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.stopListening();
}

void loop() {

  // READ CONTROLS
  data.throttle = 1023 - analogRead(LjoyY);
  data.yaw      = 1023 - analogRead(LjoyX);
  data.pitch    = analogRead(RjoyY);
  data.roll     = 1023 - analogRead(RjoyX);

  // SEND CONTROL
  bool success = radio.write(&data, sizeof(data));
  analogWrite(LED, success*30); //the led is too bright at full signal

  // RECEIVE TELEMETRY (from ACK)
  if (radio.isAckPayloadAvailable()) {
    radio.read(&telemetry, sizeof(telemetry));

    static unsigned long lastPrint = 0;

    if (millis() - lastPrint > 20) {
      Serial.print("ARM: "); Serial.print(telemetry.armed);
      Serial.print(" | T: "); Serial.print(telemetry.throttle);
      Serial.print(" | P: "); Serial.print(telemetry.pitch);
      Serial.print(" | R: "); Serial.print(telemetry.roll);
      Serial.print(" | Y: "); Serial.print(telemetry.yawRate);
      Serial.print(" | M: ");
      Serial.print(telemetry.m1); Serial.print(" ");
      Serial.print(telemetry.m2); Serial.print(" ");
      Serial.print(telemetry.m3); Serial.print(" ");
      Serial.print(telemetry.m4);
      Serial.print(" | BatV: ");
      Serial.println(telemetry.batV);

      lastPrint = millis();
      if (telemetry.batV < 11.0 && success) {
        digitalWrite(BATLED, 1);
      } else {
        digitalWrite(BATLED, 0);
      }
    }
  }

  delay(5);
}