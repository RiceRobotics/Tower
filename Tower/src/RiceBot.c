/*
 * Welcome to the RiceBot library!
 *
 * This library was written for use by the Rice University Vex U Robotics team.
 * All are welcome to use and modify the library, so long as due credit is given to the creator.
 * If you have questions/comments/suggestions, email me at Keiko.F.Kaplan@rice.edu
 *
 * This library was written to be used with the Purdue Robotic Operating System.
 *
 * Author: Keiko Kaplan
 */

#include "main.h"

//Allocates memory and initializes a Motor type.
Motor *newMotor() {
	Motor *m = malloc(sizeof(Motor));
	m->port = 0;
	m->out = 0;
	m->reflected = 1;
	return m;
}

/*
 * A simple function to set all 3 fields of a Motor.
 *
 * @param m The motor struct to be initialized
 * @param port The port on the Cortex which the motor is plugged into
 * @param out The power output to the motor, between -127 and 127
 * @param reflected If the output to the motor should be reversed. -1 or 1
 */
void initMotor(Motor *m, unsigned char port, int out, int reflected) {
	//	printf("initMotor ");
	m->port = port;
	m->out = out;
	m->reflected = reflected;
}

//Allocates memory and initializes a Pid type.
Pid *newPid() {
	Pid *p = malloc(sizeof(Pid));
	p->running = 0;
	p->setPoint = 0;
	p->current = 0;
	p->error = 0;
	p->lastError = 0;
	p->integral = 0;
	p->derivative = 0;
	p->kP = 0;
	p->kI = 0;
	p->kD = 0;
	p->output = 0;
	return p;
}

/*
 * A simple function to set the fields of a Pid type
 *
 * @param setPoint The target value for the loop
 * @param *current A pointer to relevant sensor value (CANNOT BE AN ARRAY)
 * @param error The difference between setPoint and &current
 * @param lastError The previous error value, used for derivative calculations
 * @param integral A running sum of all previous errors
 * @param derivative The difference between lastError and error
 * @param kP The coefficient for the proportional term
 * @param kI The coefficient for the integral term
 * @param kD The coefficient for the derivative term
 * @param output The value to be set to the motors
 */
void initPid(Pid *p, float kP, float kI, float kD) {
	p->running = 0;
	p->setPoint = 0;
	p->current = 0;
	p->error = 0;
	p->lastError = 0;
	p->integral = 0;
	p->derivative = 0;
	p->kP = kP;
	p->kI = kI;
	p->kD = kD;
	p->output = 0;
}

Ricencoder *newEncoder() {
	Ricencoder *r = malloc(sizeof(Ricencoder));
	r->left = 0;
	r->right = 0;
	r->ticksPerRev = 0;
	r->mult = 1;
	r->isIME = 0;
	return r;
}

/*
 * The Ricencoder contains data for a Left/Right encoder pair.
 *
 * @param ticksPerRev The number of ticks per revolution of the encoder
 * 						627.2 for the 393 IME in high torque mode (factory default)
 * 						392 for the 393 IME in high speed mode
 * 						360 for the Quadrature Encoder
 * @param mult A multiplier to use as compensation for gear ratio
 * @param isIME 1 if IME, 0 if quad encoder
 */
void initRicencoder(Ricencoder *r, float ticksPerRev, int mult, int isIME) {
	r->left = 0;
	r->right = 0;
	r->ticksPerRev = ticksPerRev;
	r->mult = mult;
	r->isIME = isIME;
}

void riceBotInitializeIO() {

}

/*
 * Call this from the default Initialize function.
 * After, be sure to reinitialize each motor you will be using on your robot.
 */
void riceBotInitialize() {

	Motor *DTFrontRight = newMotor();
	Motor *DTFrontMidRight = newMotor();
	Motor *DTMidRight = newMotor();
	Motor *DTBackRight = newMotor();
	Motor *DTFrontLeft = newMotor();
	Motor *DTFrontMidLeft = newMotor();
	Motor *DTMidLeft = newMotor();
	Motor *DTBackLeft = newMotor();

	Motor *ARMRight = newMotor();
	Motor *ARMLeft = newMotor();
	Motor *ARMTopRight = newMotor();
	Motor *ARMBottomRight = newMotor();
	Motor *ARMTopLeft = newMotor();
	Motor *ARMBottomLeft = newMotor();

	Motor *Collector = newMotor();

	printf("Motors are go ");

//	imeInitializeAll();
	printf("imes init ");
	encDTLeft = encoderInit(0, 0, false);
	encDTRight = encoderInit(0, 0, false);
	encARMLeft = encoderInit(0, 0, false);
	encARMRight = encoderInit(0, 0, false);

	Ricencoder *ENCDT = newEncoder();
	Ricencoder *ENCARM = newEncoder();
	printf("Encoders setup");
	gyro = gyroInit(0, 196);
	gyroVal = gyroGet(gyro);

	analogCalibrate(POTARMLeft);
	analogCalibrate(POTARMRight);

	Pid *PidARMLeft = newPid();
	Pid *PidARMRight = newPid();
	printf("Pid types made");
}

/*
 * Checks joystick input and sets all Motor structs to appropriate output
 * @param controlStyle The format of the joystick input.
 * 			Can be:
 * 		 			TANKDRIVE
 * 	 	 			ARCADEDRIVE
 *	 	 			CHEEZYDRIVE
 *	 	 			MECANUMDRIVE
 *	 	 			HDRIVE
 */
void getJoystickForDriveTrain(int controlStyle) {
	int x1 = joystickGetAnalog(1, 4);
	int y1 = joystickGetAnalog(1, 3);
	int x2 = joystickGetAnalog(1, 1);
	int y2 = joystickGetAnalog(1, 2);

	switch(controlStyle) {
	case CTTANKDRIVE:
		DTFrontLeft.out = y1;
		DTFrontMidLeft.out = y1;
		DTMidLeft.out = y1;
		DTBackLeft.out = y1;

		DTFrontRight.out = y2;
		DTFrontMidRight.out = y2;
		DTMidRight.out = y2;
		DTBackRight.out = y2;
		break;
	case CTARCADEDRIVE:
		DTFrontLeft.out = (y1 + x1) / 2;
		DTFrontMidLeft.out = (y1 + x1) / 2;
		DTMidLeft.out = (y1 + x1) / 2;
		DTBackLeft.out = (y1 + x1) / 2;

		DTFrontRight.out = (y1 - x1) / 2;
		DTFrontMidRight.out = (y1 - x1) / 2;
		DTMidRight.out = (y1 - x1) / 2;
		DTBackRight.out = (y1 - x1) / 2;
		break;
	case CTCHEEZYDRIVE:
		DTFrontLeft.out = (y1 + x2) / 2;
		DTFrontMidLeft.out = (y1 + x2) / 2;
		DTMidLeft.out = (y1 + x2) / 2;
		DTBackLeft.out = (y1 + x2) / 2;

		DTFrontRight.out = (y1 - x2) / 2;
		DTFrontMidRight.out = (y1 - x2) / 2;
		DTMidRight.out = (y1 - x2) / 2;
		DTBackRight.out = (y1 - x2) / 2;
		break;
	case CTMECANUMDRIVE:
		DTFrontLeft.out = y1 + x2 + x1;
		DTBackLeft.out = y1 + x2 - x1;

		DTFrontRight.out = y1 - x2 - x1;
		DTBackRight.out = y1 - x2 + x1;
		break;
	case CTHDRIVE:
	default:
		break;
	}
}

/* Final stage: sets all physical motors based on output set in Motor structs
 * Run in a task?
 * @param driveTrainStyle The configuration of the wheels on the robot.
 * 			Can be:
 * 					FOURWHEELS
 * 					SIXWHEELS
 * 					EIGHTWHEELS
 * 					MECANUM
 * 					HOLONOMIC
 * 					HDRIVE
 * 					SWERVE
 */
void setDriveTrainMotors(int driveTrainStyle) {
	switch(driveTrainStyle) {
	case DTFOURWHEELS:
		motorSet(DTFrontLeft.port, DTFrontLeft.out * DTFrontLeft.reflected);
		motorSet(DTBackLeft.port, DTBackLeft.out * DTBackLeft.reflected);

		motorSet(DTFrontRight.port, DTFrontRight.out * DTFrontRight.reflected);
		motorSet(DTBackRight.port, DTBackRight.out * DTBackRight.reflected);
		break;
	case DTSIXWHEELS:
		motorSet(DTFrontLeft.port, DTFrontLeft.out * DTFrontLeft.reflected);
		motorSet(DTMidLeft.port, DTMidLeft.out * DTMidLeft.reflected);
		motorSet(DTBackLeft.port, DTBackLeft.out * DTBackLeft.reflected);

		motorSet(DTFrontRight.port, DTFrontRight.out * DTFrontRight.reflected);
		motorSet(DTMidRight.port, DTMidRight.out * DTMidRight.reflected);
		motorSet(DTBackRight.port, DTBackRight.out * DTBackRight.reflected);
		break;
	case DTEIGHTWHEELS:
		motorSet(DTFrontLeft.port, DTFrontLeft.out * DTFrontLeft.reflected);
		motorSet(DTFrontMidLeft.port, DTFrontMidLeft.out * DTFrontMidLeft.reflected);
		motorSet(DTMidLeft.port, DTMidLeft.out * DTMidLeft.reflected);
		motorSet(DTBackLeft.port, DTBackLeft.out * DTBackLeft.reflected);

		motorSet(DTFrontRight.port, DTFrontRight.out * DTFrontRight.reflected);
		motorSet(DTFrontMidRight.port, DTFrontMidRight.out * DTFrontMidRight.reflected);
		motorSet(DTMidRight.port, DTMidRight.out * DTMidRight.reflected);
		motorSet(DTBackRight.port, DTBackRight.out * DTBackRight.reflected);
		break;
	default:
		break;
	}
}

int autonomousTask(int instruction, int distance, int pow, long timeout) {
	int success = 0;
	long startTime = millis();

	int power[2];
	power[1] = (pow == NULL) ? 127 : pow;
	power[0] = power[1];

	switch(instruction) {
	case AUTODRIVEBASIC:;
	int target;
	target = ENCDT.ticksPerRev / (4 * MATH_PI) * distance;
	//		power = (pow == NULL) ? 127 : pow;
	int current[2] = {ENCDT.left, ENCDT.right};

	while(current[1] < target && millis() < startTime + timeout) {
		if(abs(current[1] - current[0]) > 50) {
			if(current[0] > current[1]) {
				power[0] = speedRegulator(power[0] - 2);
			} else if(current[0] < current[1]) {
				power[0] = speedRegulator(power[0] + 2);
			}
		}

		DTFrontRight.out = power[1];
		DTFrontMidRight.out = power[1];
		DTMidRight.out = power[1];
		DTBackRight.out = power[1];
		DTFrontLeft.out = power[0];
		DTFrontMidLeft.out = power[0];
		DTMidLeft.out = power[0];
		DTBackLeft.out = power[0];

		delay(20);
		current[0] = ENCDT.left;
		current[1] = ENCDT.right;
	}
	break;
	}
	return success;
}

void processPid(Pid *pidLoop, int current) {
	if (pidLoop->running) {
		pidLoop->current = current;
		//	printf("Current: %d\n\r", pidLoop->current);
		pidLoop->error = pidLoop->setPoint - pidLoop->current;
		pidLoop->integral += pidLoop->error;
		pidLoop->derivative = pidLoop->lastError - pidLoop->error;

		pidLoop->output = speedRegulator((pidLoop->error * pidLoop->kP) +
				(pidLoop->integral * pidLoop->kI) + (pidLoop->derivative * pidLoop->kD));

		pidLoop->lastError = pidLoop->error;
	}

}

int speedRegulator(int speed) {
	if(speed > 127) {
		return 127;
	} else if(speed < -127) {
		return -127;
	} else {
		return speed;
	}
}

int max(int a, int b) {
	if(a < b) {
		return b;
	}
	return a;
}

int min(int a, int b) {
	if(a > b) {
		return b;
	}
	return a;
}
