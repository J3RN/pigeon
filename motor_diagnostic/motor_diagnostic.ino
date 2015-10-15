#include <Servo.h>

#define PIN 4

Servo motor;
int speed;

void setup() {
  Serial.begin(115200);

  Serial.println(F("Enter any character to begin initialization"));
  while (!Serial.available()); Serial.read();

  motor.attach(PIN);
  delay(3000);

  motor.write(10);
  delay(3000);

  Serial.println(F("Enter beginning speed"));
  while (!Serial.available());

  speed = Serial.readString().toInt();

  Serial.println(F("Beginning..."));
  delay(3000);

  motor.write(speed);
}

void loop() {
  Serial.println(F("Enter a new speed"));
  while (!Serial.available());
  speed = Serial.readString().toInt();
  motor.write(speed);
}
