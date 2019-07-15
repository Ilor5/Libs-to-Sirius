//Либа для расчетов
#include "calculation.h" 
void pwmValue(float k, float omega, float induction, int sign, uint8_t *result, int *direction) { //расчет ШИМ
	*result = roundf((k * resistance * (sign * omega * induction)) / (area * coil));
	*result = roundf((*result / 1.2) * 255);
	*direction = 0;
	if (*result < -255) {
		*result = -255;
	}
        if (result < 0) {
		*result = 255 + *result;
		*direction = 1;
	}
	if (*result > 255) {
		*result = 255;
	}
}