//Либа для расчетов
#include "calculation.h"
#include "math.h"

uint8_t m_axis(float k, float omega, float B_axis, int *dir) {
	*dir = 0;
	if (omega <= stopVelocity) {
		return 0;
	}
	float bidot = k * (omega * B_axis);
	if (fabs(bidot) > maxVoltage) {
		bidot = maxVoltage * ((bidot > 0) ? 1 : -1) * 255;
	} else {
		bidot = (bidot / maxVoltage) * 255;
	}
	if (bidot < 0) {
		bidot += 255;
		*dir = 1;
	}
	return (uint8_t)bidot;
}

void getVoltage(float omega, float B[3], float angle, float *voltageX, float *voltageY) {
	if ((omega >= stopVelocity) || (-omega >= stopVelocity)) {
		*voltageX = kOmega * omega * (-B[1]);
		*voltageY = kOmega * omega * B[0];
	} else {
		*voltageX += kAlpha * sin(angleReq - angle) * (-B[1]);
		*voltageY += kAlpha * sin(angleReq - angle) * B[0];
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
