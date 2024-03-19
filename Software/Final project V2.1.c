// Version 21

#include "mindsensors-motormux.h"

void configureAllSensors();
int getFullxTime();
void startUp();
void startPosition();
void latheOperation();
void yMotion_toWhite();
void yMotion_toBlack();
void resetPosition(int full_x_time);
void spindleFirstPass();
void spindleFullCut();

const int DOOR_DISTANCE = 25; // distance of the case from the ultrasonic
const int Y_WAIT_TIME = 5;
const int X_DRIVE_SPEED = 20;
const int SPINDLE_DIM_TIME = 350;
const int ROTATION_TIME = 300;

/* Assume
M1 multiplex S4 sensor: x-axis motor
MotorA: spindle motor
MotorB: y-axis motor
MotorC and MotorD: foam rotation. C will be left, D will be right.
S1 sensor: color
S2 sensor: ultrasonic
S3 sensor: touch
*/

task main()
{
	configureAllSensors();

	startUp();
	int full_x_time = 0;
	full_x_time = getFullxTime();

	// Start spinning spindle until the end of latheOperation.
	motor[motorA] = 100;

	startPosition();

	while (SensorValue[S3] == 0 && SensorValue[S2] < DOOR_DISTANCE)
	{
		latheOperation();
	}

	// Stop rotation of spindle.
	motor[motorA] = 0;

  // Stop any rotation of object.
	motor[motorC] = motor[motorD] = 0;

	// Stop x-axis motor just to be sure it is stopped before return.
	MSMotorStop(mmotor_S4_1);

	resetPosition(full_x_time);

	// An extra MSMotorStop for redundancy.
	MSMotorStop(mmotor_S4_1);
}


void startUp()
{
	// Wait until box is on top.
	while (SensorValue[S2] > DOOR_DISTANCE)
	{}

	// Wait 5 seconds for safety.
	clearTimer(T3);
	while (time1[T3] < 5000)
	{}
}

int getFullxTime()
{
	int x_time = 0;
	clearTimer(T1);

	// Drive to far right at drive speed and time it.
	MSMMotor(mmotor_S4_1, -X_DRIVE_SPEED);
	while(!SensorValue[S3])
	{}
	MSMotorStop(mmotor_S4_1);

	// Make the return time a little bit less.
	x_time = time1[T1] - 500;

	// Wait for momentum to go to 0 before reversing.
	clearTimer(T2);
	while (time1[T2] < 100)
	{}

	// Drive back to far left at drive speed using the time you got.
	clearTimer(T1);
  MSMMotor(mmotor_S4_1, X_DRIVE_SPEED);
  while (time1[T1] < x_time)
 	{}
	MSMotorStop(mmotor_S4_1);

	return x_time;
}


void startPosition()
{
	/* After this function has been called at the beginning, you are now on
	the bottom side of the line. You are on the "right" side if looking
	down the x-axis, i.e. eyes toward the END touch sensor. */

	/* Assume that the program started with the color sensor as far away
	from the foam as possible. */

	nMotorEncoder[motorB] = 0;

	motor[motorB] = -20;
	while (SensorValue[S1] != (int)colorBlack)
	{}

	motor[motorB] = 0;
}

void latheOperation()
{

	yMotion_toWhite();

	// Timer 1 is for moving along x at a precise distance
	clearTimer(T1);

	// Move x the distance of the spindle diameter.
	MSMMotor(mmotor_S4_1, -5);
	while(time1[T1] < SPINDLE_DIM_TIME)
	{}
	MSMotorStop(mmotor_S4_1);

	yMotion_toBlack();

	// Do first pass with in and outs.
	spindleFirstPass();

	// Do full pass with spindle fully in.
	spindleFullCut();
}

void yMotion_toWhite()
{
  // Go to furthest y-axis point from object.
  motor[motorB] = 5;

  while (nMotorEncoder[motorB] < 0)
  {}

  motor[motorB] = 0;
	clearTimer(T1);
	while(time1[T1] < Y_WAIT_TIME)
	{}
}

void yMotion_toBlack()
{
	// Go to object (black).
  motor[motorB] = -5;
	while (SensorValue[S1] != (int)colorBlack)
  {}

	motor[motorB] = 0;
  clearTimer(T1);
  while(time1[T1] < Y_WAIT_TIME)
  {}
}

void spindleFirstPass()
{
	nMotorEncoder[motorC] = 0;
	while(nMotorEncoder[motorC] > -360 && SensorValue[S2] < DOOR_DISTANCE)
	{
		yMotion_toWhite();
		motor[motorC] = motor[motorD] = -5;
		clearTimer(T2);
		while (time1[T2] < ROTATION_TIME)
		{}
		motor[motorC] = motor[motorD] = 0;
		yMotion_toBlack();
	}
}

void spindleFullCut()
{
	nMotorEncoder[motorC] = 0;
	while (nMotorEncoder[motorC] > -360 && SensorValue[S2] < DOOR_DISTANCE)
	{
		motor[motorC] = motor[motorD] = -5;
		clearTimer(T2);
		while (time1[T2] < ROTATION_TIME)
		{}
		motor[motorC] = motor[motorD] = 0;
		// Waits as it cuts before rotating again, with spindle staying in.
		while (time1[T2] < 1500)
		{}
	}
}

void resetPosition(int full_x_time)
{
	// Retract spindle to furthest point from the object.
  motor[motorB]= 40;
  while (nMotorEncoder[motorB] < 0)
  {}
  motor[motorB] = 0;

  // Drive back to far left.
	clearTimer(T1);
  MSMMotor(mmotor_S4_1, X_DRIVE_SPEED);
  while (time1[T1] < full_x_time)
 	{}
	MSMotorStop(mmotor_S4_1);
}

void configureAllSensors()
{
    SensorType[S1] = sensorEV3_Color;
    wait1Msec(50);
    SensorMode[S1] = modeEV3Color_Color;
    wait1Msec(50);
    SensorType[S2] = sensorEV3_Ultrasonic;
    SensorType[S3] = sensorEV3_Touch;
    SensorType[S4] = sensorI2CCustom;
    MSMMUXinit();
    wait1Msec(50);
}
