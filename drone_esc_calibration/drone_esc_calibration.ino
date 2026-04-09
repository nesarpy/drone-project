#include <Servo.h>

Servo FL, FR, BL, BR;

void setup() {
  FL.attach(3);
  FR.attach(5);
  BL.attach(6);
  BR.attach(9);

  // Send MAX throttle
  FL.writeMicroseconds(2000);
  FR.writeMicroseconds(2000);
  BL.writeMicroseconds(2000);
  BR.writeMicroseconds(2000);

  delay(2000); // wait for ESC beeps

  // Send MIN throttle
  FL.writeMicroseconds(1000);
  FR.writeMicroseconds(1000);
  BL.writeMicroseconds(1000);
  BR.writeMicroseconds(1000);

  delay(3000);
}

void loop() {}