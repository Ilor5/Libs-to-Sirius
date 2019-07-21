//Либа для расчетов
#include "calculation.h"

/*void getVoltage(float omega, float B[3], float angle, float *voltageX, float *voltageY) {
	if ((omega >= stopVelocity) || (-omega >= stopVelocity)) {
		*voltageX = kBdot * omega * (-B[1]);
		*voltageY = kBdot * omega * B[0];
	} else {
		*voltageX = kOmega * omega * (-B[1]) - kAlpha * sin(angleReq - angle) * (-B[1]);
		*voltageY = kOmega * omega * B[0] - kAlpha * sin(angleReq - angle) * B[0];
	}
}*/

void getVoltageBidot(float omega, float B[3], float *voltageX, float *voltageY, float stopVelocity) {
	if ((omega >= stopVelocity) || (-omega >= stopVelocity)) {
		*voltageX = kBdot * omega * (-B[1]);
		*voltageY = kBdot * omega * B[0];
	} else {
          *voltageX = 0;
          *voltageY = 0;
        }
}

void getVoltagePointing(float omega, float B[3], float angle, float *voltageX, float *voltageY, float stopAngle, int *status) {
	if ((omega >= defaultStopVelocity) || (-omega >= defaultStopVelocity)) {
		*voltageX = kBdot * omega * (-B[1]);
		*voltageY = kBdot * omega * B[0];
                *status = 0;
	} else {
		*voltageX = kOmega * omega * (-B[1]) - kAlpha * sin(stopAngle - angle) * (-B[1]);
		*voltageY = kOmega * omega * B[0] - kAlpha * sin(stopAngle - angle) * B[0];
                *status = 1;
	}
}

void scaleVoltage(float *voltageX, float *voltageY) {
	float absVoltageX = fabs(*voltageX);
	float absVoltageY = fabs(*voltageY);
	float scale = 1;
	if ((absVoltageX >= maxVoltage) || (absVoltageY >= maxVoltage)) {
          if ((absVoltageX / maxVoltage) >= (absVoltageY / maxVoltage)){
			scale = absVoltageX / maxVoltage;
	} else {
		scale = absVoltageY / maxVoltage;
	}
	*voltageX /= scale;
	*voltageY /= scale;
        }
}

void getPWM(float voltage, int *dir, uint8_t *pwm) {
	*dir = 0;
   int _pwm = 0;
	if (fabs(voltage) > maxVoltage) {
		_pwm = roundf(maxVoltage * ((voltage > 0) ? 1 : -1) * 255);
	} else {
		_pwm = roundf((voltage / maxVoltage) * 255);
	}
	if (_pwm < 0) {
		_pwm += 255;
		*dir = 1;
	}
   *pwm = (uint8_t)_pwm;
}









