#define SIGNAL_PIN 10

void setup() {
  Serial.begin(115200);
}

void loop() {
  if (digitalRead(SIGNAL_PIN)) {
    Serial.println(F("Signal!"));
  }
}
