#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>

#define BL 2
#define BR 3
#define TR 4
#define TL 5

#define START_SPEED 90
#define MAX_SPEED 179
#define MIN_SPEED 70

/* Assign a unique ID to the sensors */
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);

int MOTOR_PINS[4]   = { BL, BR, TR, TL };
int MOTOR_SPEEDS[4];
Servo MOTORS[4];

/* PID controller constants */
double Kp = 1, Ki = 0, Kd = 0;

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
  Serial.print(F("Pitch: "));
  Serial.print(pitchOut);
  Serial.print(F(" Roll: "));
  Serial.println(rollOut);

  /* Calculate new motor speeds */
  MOTOR_SPEEDS[TL] = saneSpeed(MOTOR_SPEEDS[TL], -pitchOut - rollOut);
  MOTOR_SPEEDS[TR] = saneSpeed(MOTOR_SPEEDS[TR], -pitchOut + rollOut);
  MOTOR_SPEEDS[BL] = saneSpeed(MOTOR_SPEEDS[BL], pitchOut - rollOut);
  MOTOR_SPEEDS[BR] = saneSpeed(MOTOR_SPEEDS[BR], pitchOut + rollOut);

  /* Write speeds */
  int i;
  for (i = 0; i < 4; i++) {
    MOTORS[i].write(MOTOR_SPEEDS[i]);
  }
}

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
}

void attachMotors() {
  /* Attach motors */
  int i;
  for (i = 0; i < 4; i++) {
    MOTORS[i].attach(MOTOR_PINS[i]);
  }
  delay(1000);
}

void initMotors() {
  int i;
  for (i = 0; i < 4; i++) {
    MOTORS[i].write(10);
  }
  delay(3000);
}

void printResults() {
  /* Print results */
  Serial.println(F("Motor Speeds:"));

  Serial.print(F("TL: "));
  Serial.print(MOTOR_SPEEDS[TL]);
  Serial.print(F(" TR: "));
  Serial.print(MOTOR_SPEEDS[TR]);
  Serial.print(F(" BL: "));
  Serial.print(MOTOR_SPEEDS[BL]);
  Serial.print(F(" BR: "));
  Serial.print(MOTOR_SPEEDS[BR]);
  Serial.println(F(""));
}

/**************************************************************************/
/*!
  @brief  Start the motors and sensors
 */
/**************************************************************************/
void setup(void) {
  Serial.begin(115200);

  /* Initialise the sensors */
  initSensors();

  /* Initialize the PID controllers */
  initControllers();

  /* Attach motors */
  attachMotors();

  /* Initialize motors */
  initMotors();

  /* Calculate end time (now + 5 seconds) */
  endtime = millis() + 5000;

  /* Turn PIDs on */
  pitchController.SetMode(AUTOMATIC);
  rollController.SetMode(AUTOMATIC);

  /* Start motors (panic! panic!) */
  int i;
  for (i = 0; i < 4; i++) {
    MOTOR_SPEEDS[i] = START_SPEED;
    MOTORS[i].write(START_SPEED);
  }
}

/**************************************************************************/
/*!
  @brief  Constantly check the pitch and roll and recalculate
 */
/**************************************************************************/
void loop(void) {
  /* Kill */
  if (millis() > endtime) {
    int i;
    for (i = 0; i < 4; i++) {
      MOTORS[i].write(0);
    }

    printResults();

    /* Die */
    while(1);
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
  }
}
