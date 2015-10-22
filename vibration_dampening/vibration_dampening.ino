#include <Wire.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>

#define BL 0
#define BR 1
#define TR 2
#define TL 3

#define BL_PIN 2
#define BR_PIN 3
#define TR_PIN 4
#define TL_PIN 5

#define START_SPEED 75

#define RUN_TIME 10000

#define INIT_PIN  13
#define TEST_PIN  12
#define START_PIN 11
#define STOP_PIN  10

/* Assign a unique ID to the sensors */
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);

int MOTOR_PINS[4]   = { BL_PIN, BR_PIN, TR_PIN, TL_PIN };
int MOTOR_SPEEDS[4];
Servo MOTORS[4];

/* Time counter */
unsigned long endtime;

/**************************************************************************/
/*!
  @brief  Initialises the accelerometer
 */
/**************************************************************************/
void initSensors() {
  if(!accel.begin()) {
    Serial.println(F("No accelerometer detected"));
    while(1);
  }
}

/**************************************************************************/
/*!
  @brief  "Attach" each motor to it's pin
 */
/**************************************************************************/
void attachMotors() {
  /* Attach motors */
  int i;
  for (i = 0; i < 4; i++) {
    MOTORS[i].attach(MOTOR_PINS[i]);
  }
  delay(1000);
}

/**************************************************************************/
/*!
  @brief  Write a low speed to each motor to initialize it and pause
 */
/**************************************************************************/
void initMotors() {
  int i;
  for (i = 0; i < 4; i++) {
    MOTORS[i].write(10);
  }
  delay(3000);
}

/**************************************************************************/
/*!
  @brief  Set the speed for every motor to the given speed
 */
/**************************************************************************/
void setAllSpeeds(int speed) {
  int i;
  for (i = 0; i < 4; i++) {
    MOTOR_SPEEDS[i] = speed;
    MOTORS[i].write(speed);
  }
}

/**************************************************************************/
/*!
  @brief  Set all the motors to their starting speed for three seconds
 */
/**************************************************************************/
void testMotors() {
  setAllSpeeds(START_SPEED);
  delay(3000);
  setAllSpeeds(0);
}

/**************************************************************************/
/*!
  @brief  Start the motors and sensors
 */
/**************************************************************************/
void setup() {
  Serial.begin(115200);

  while (!digitalRead(INIT_PIN));

  /* Initialise the sensors */
  initSensors();

  /* Attach motors */
  attachMotors();

  /* Initialize motors */
  initMotors();

  while (!digitalRead(START_PIN)) {
    // Optionally test motors
    if (!digitalRead(TEST_PIN)) {
      testMotors();
    }
  }

  /* Calculate end time */
  endtime = millis() + RUN_TIME;

  Serial.println(F("X,Y"));

  /* Start motors (panic! panic!) */
  setAllSpeeds(START_SPEED);
}

/**************************************************************************/
/*!
  @brief  Constantly check the pitch and roll and print them
 */
/**************************************************************************/
void loop() {
  /* Stop */
  if (millis() > endtime || digitalRead(STOP_PIN)) {
    int i;

    for (i = 0; i < 4; i++) {
      MOTORS[i].write(0);
    }
  }

  sensors_event_t accel_event;
  sensors_vec_t   orientation;

  /* Compute PID output from orientation */
  accel.getEvent(&accel_event);
  if (dof.accelGetOrientation(&accel_event, &orientation)) {
    Serial.print(orientation.pitch);
    Serial.print(",");
    Serial.print(orientation.roll);
  }
}
