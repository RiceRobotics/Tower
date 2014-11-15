/*
 * MotorTask.c
 *
 *  Created on: Oct 27, 2014
 *      Author: Keiko
 */

#include "main.h"

void startIOTask(void *ignore) {
	while(1) {
//		printf("IOTask ");
		setDriveTrainMotors(DTFOURWHEELS);

		motorSet(ARMLeft.port, ARMLeft.out * ARMLeft.reflected);
		motorSet(ARMRight.port, ARMRight.out * ARMRight.reflected);
		motorSet(ARMTopLeft.port, ARMTopLeft.out * ARMTopLeft.reflected);
		motorSet(ARMTopRight.port, ARMTopRight.out * ARMTopRight.reflected);
		motorSet(ARMBottomLeft.port, ARMBottomLeft.out * ARMBottomLeft.reflected);
		motorSet(ARMBottomRight.port, ARMBottomRight.out * ARMBottomRight.reflected);

		motorSet(Collector.port, Collector.out * Collector.reflected);

		armPot[0] = analogReadCalibrated(POTARMLeft);
		armPot[1] = -analogReadCalibrated(POTARMRight);

//		if(ENCDT.isIME) {
//			imeGet(IMEDTLEFT, &ENCDT.left);
//			imeGet(IMEDTRIGHT, &ENCDT.right);
//		} else {
//			ENCDT.left = encoderGet(encDTLeft);
//			ENCDT.right = encoderGet(encDTRight);
//		}
//
//		if(ENCARM.isIME) {
//			imeGet(IMEDTLEFT, &ENCARM.left);
//			imeGet(IMEDTRIGHT, &ENCARM.right);
//		} else {
//			ENCARM.left = encoderGet(encARMLeft);
//			ENCARM.right = encoderGet(encARMRight);
//		}
		gyroVal = gyroGet(gyro);

		delay(10);
	}
}

void startPidTask(void *ignore) {
	while(1) {
//		printf("PidTask\n\r");
		//Manually add each pid loop here
		processPid(&PidARMLeft, (armPot[0]));
		processPid(&PidARMRight, (armPot[1]));
		if(PidARMLeft.running && PidARMRight.running) {
			ARMBottomLeft.out = PidARMLeft.output;
			ARMBottomRight.out = PidARMRight.output;
		}

		printf("SetPoint: %d/%d, Pot L/R: %d/%d, MotorOut %d/%d Running: %d/%d \n\r", PidARMLeft.setPoint,
				PidARMRight.setPoint, PidARMLeft.current, PidARMRight.current, ARMBottomLeft.out, ARMBottomRight.out,
				PidARMLeft.running, PidARMRight.running);

		delay(20);
	}
}
