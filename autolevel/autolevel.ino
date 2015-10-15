#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>
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
#define MAX_SPEED 130
#define MIN_SPEED 70

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

/* PID controller constants */
double Kp = 0.009, Ki = 0, Kd = 0;

double setpoint;

/* Pitch PID controller */
double pitchIn, pitchOut;
PID pitchController(&pitchIn, &pitchOut, &setpoint, Kp, Ki, Kd, DIRECT);

/* Roll PID controller */
double rollIn, rollOut;
PID rollController(&rollIn, &rollOut, &setpoint, Kp, Ki, Kd, DIRECT);

/* Time counter */
unsigned long endtime;

/**************************************************************************/
/*!
  @brief  Initialises all the sensors used by this example
 */
/**************************************************************************/
void initSensors() {
  if(!accel.begin()) {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no accelerometer detected ... Check your wiring!"));
    while(1);
  }
}

/**************************************************************************/
/*!
  @brief  Return a speed guaranteed to be within the range
    MIN_SPEED - MAX_SPEED
 */
/**************************************************************************/
int saneSpeed(int base, int change) {
  if ((base + change) > MAX_SPEED) {
    return MAX_SPEED;
  } else if ((base + change) < MIN_SPEED) {
    return MIN_SPEED;
  } else {
    return base + change;
  }
}

/**************************************************************************/
/*!
  @brief  Adjust all motors based on PID output
 */
/**************************************************************************/
void adjustMotors() {
  /* Calculate new motor speeds */
  MOTOR_SPEEDS[BL] = saneSpeed(MOTOR_SPEEDS[BL], (pitchOut - rollOut) / 2);
  MOTOR_SPEEDS[BR] = saneSpeed(MOTOR_SPEEDS[BR], (pitchOut + rollOut) / 2);
  MOTOR_SPEEDS[TR] = saneSpeed(MOTOR_SPEEDS[TR], (-pitchOut + rollOut) / 2);
  MOTOR_SPEEDS[TL] = saneSpeed(MOTOR_SPEEDS[TL], (-pitchOut - rollOut) / 2);

  /* Write speeds */
  int i;
  for (i = 0; i < 4; i++) {
    MOTORS[i].write(MOTOR_SPEEDS[i]);
  }
}

/**************************************************************************/
/*!
  @brief  Configure the PID controllers (limits, setpoints, and sample times)
 */
/**************************************************************************/
void initControllers() {
  sensors_event_t accel_event;
  sensors_vec_t   orientation;

  accel.getEvent(&accel_event);
  if (dof.accelGetOrientation(&accel_event, &orientation))
  {
    rollIn = orientation.roll;
    pitchIn = orientation.pitch;
  }

  /* Initialize the controllers */
  setpoint = 0;

  /* Sensible limits */
  rollController.SetOutputLimits(-80, 80);
  pitchController.SetOutputLimits(-80, 80);

  /* More appropriate sample time */
  rollController.SetSampleTime(5);
  pitchController.SetSampleTime(5);
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
  @brief  Write a low speed to each motor to initialize it
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

  /* Initialize the PID controllers */
  initControllers();

  /* Attach motors */
  attachMotors();

  /* Initialize motors */
  initMotors();

  while (!digitalRead(START_PIN)) {
    // Optionally Test motors
    if (!digitalRead(TEST_PIN)) {
      testMotors();
    }
  }

  /* Calculate end time */
  endtime = millis() + RUN_TIME;

  /* Turn PIDs on */
  pitchController.SetMode(AUTOMATIC);
  rollController.SetMode(AUTOMATIC);

  /* Start motors (panic! panic!) */
  setAllSpeeds(START_SPEED);
}

/**************************************************************************/
/*!
  @brief  Constantly check the pitch and roll and recalculate
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
    pitchIn = orientation.pitch;
    rollIn = orientation.roll;

    pitchController.Compute();
    rollController.Compute();

    adjustMotors();

    // Print data
    Serial.print(F("Pitch: "));
    Serial.print(pitchIn);
    Serial.print(F(" Roll: "));
    Serial.print(rollIn);

    Serial.print(F(" PC: "));
    Serial.print(pitchOut);
    Serial.print(F(" RC: "));
    Serial.print(rollOut);

    Serial.print(" BL: ");
    Serial.print(MOTOR_SPEEDS[BL]);
    Serial.print(" BR: ");
    Serial.print(MOTOR_SPEEDS[BR]);
    Serial.print(" TR: ");
    Serial.print(MOTOR_SPEEDS[TR]);
    Serial.print(" TL: ");
    Serial.print(MOTOR_SPEEDS[TL]);
    Serial.println();
  }
}
