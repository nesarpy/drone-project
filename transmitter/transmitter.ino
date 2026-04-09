#include <SPI.h>
#include <RF24.h>

RF24 radio(7, 8);
const byte address[6] = "00001";

int LjoyX = A0;
int LjoyY = A2;

int RjoyX = A4;
int RjoyY = A6;

int LED = 4;

struct DataPacket {
  int16_t throttle;
  int16_t yaw;
  int16_t pitch;
  int16_t roll;
};

DataPacket data;

void setup() {
  Serial.begin(9600);
  pinMode(LED, OUTPUT);

  radio.begin();
  radio.openWritingPipe(address);
  radio.setChannel(108);
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
  radio.stopListening();
}

void loop() {
  // Left joystick (Throttle + Yaw)
  data.throttle = 1023 - analogRead(LjoyY);
  data.yaw = 1023 - analogRead(LjoyX);
  data.roll = 1023 - analogRead(RjoyX);   

  // Deadzone fix
  if (data.throttle > 0 && data.throttle < 10) {
    data.throttle = 0;
  }

  if (data.yaw > 0 && data.yaw < 10) {
    data.yaw = 0;
  }

  // Right joystick (Pitch + Roll)
  data.pitch = analogRead(RjoyY);
  data.roll = analogRead(RjoyX);

  bool success = radio.write(&data, sizeof(data));

  // Serial.print("Throttle: ");
  // Serial.print(data.throttle);
  // Serial.print(" | Yaw: ");
  // Serial.print(data.yaw);

  // Serial.print(" | Pitch: ");
  // Serial.print(data.pitch);
  // Serial.print(" | Roll: ");
  // Serial.print(data.roll);

  // Serial.print(" | TX status: ");
  // Serial.println(success);

  digitalWrite(LED, success);
}