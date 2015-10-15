#include <Servo.h>

#define BL 2
#define BR 3
#define TR 4
#define TL 5

#define RUN 90

enum TestType { TOGETHER, INDIVIDUAL };

TestType type = INDIVIDUAL;
bool Output = true;

Servo tl;
Servo tr;
Servo br;
Servo bl;

Servo motors[4] = {tl, tr, br, bl};
String motor_names[4] = {"TL", "TR", "BR", "BL"};
int offset[4] = { 0, 0, 0, 0 };

void run_indiv() {
  for (int i = 0; i < 4; i++) {
    if (Output) {
      Serial.print("********************** TESTING MOTOR ");
      Serial.print(motor_names[i]);
      Serial.println(" ************************");
    }

    /* Start */
    motors[i].write(RUN + offset[i]);

    delay(2000);

    /* Stop */
    motors[i].write(0);

    delay(5000);
  }
}

void run_together() {
  /* Start all motors */
  for (int i = 0; i < 4; i++) {
    motors[i].write(RUN + offset[i]);
  }

  delay(2000);

  /* Stop */
  for (int i = 0; i < 4; i++) {
    motors[i].write(0);
  }
}

void setup() {
  if (Output) Serial.begin(115200);

  tl.attach(TL);
  tr.attach(TR);
  br.attach(BR);
  bl.attach(BL);

  delay(1000);

  /* Initialization */
  if (Output) Serial.println("INITIALIZATION");
  for (int i = 0; i < 4; i++) {
    motors[i].write(10);
  }
  if (Output) Serial.println("INITIALIZATION COMPLETE");

  delay(3000);
}

void loop() {
  if (Output) {
    Serial.println("Press any key to begin");
    while (!Serial.available());
    Serial.read();
  }

  if (Output) Serial.println("TESTING MOTORS");

  switch (type) {
    case TOGETHER:
      run_together();
      break;
    case INDIVIDUAL:
      run_indiv();
      break;
  }

  if (Output) Serial.println("MOTOR TEST COMPLETED");
}
