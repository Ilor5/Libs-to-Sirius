//Либа для расчетов
/*#include "calculation.h"
void pwmValue(float k, float omega, float induction, uint8_t *result, int *direction) { //расчет ШИМ
	if (omega > stopVelocity) {
		float voltage;
		*direction = 0;
		voltage = k * (omega * induction);
		if (abs(voltage) > maxVoltage) {
			voltage = maxVoltage * voltage / abs(voltage);
		}
		if (voltage < 0) {
			voltage = abs(voltage);
			*direction = 1;
			*result = roundf(255 - (voltage / maxVoltage) * 255);
		} else {
			*result = roundf((voltage / maxVoltage) * 255);
		}
	} else {
		*result = 0;
		*direction = 0;
	}
}

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
	float absVoltageX = abs(*voltageX);
	float absVoltageY = abs(*voltageY);
	float scale = 1;
	if ((absVoltageX >= maxVoltage) || (absVoltageY >= maxVoltage)) {
		if ((absVoltageX / maxVoltage) >= (absVoltageY / maxVoltage))
			scale = absVoltageX / maxVoltage;
	} else {
		scale = absVoltageY / maxVoltage;
	}
	*voltageX /= scale;
	*voltageY /= scale;
}

void getPWM(float voltage, int *dir, uint8_t *pwm) {
	*dir = 0;
	if (abs(voltage) > maxVoltage) {
		*pwm = uint8_t(maxVoltage * ((voltage > 0) ? 1 : -1) * 255);
	} else {
		*pwm = uint8_t((voltage / maxVoltage) * 255);
	}
	if (pwm < 0) {
		*pwm += 255;
		*dir = 1;
	}
}*/
