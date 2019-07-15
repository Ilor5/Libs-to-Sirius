//Либа для расчетов
#include "math.h"
#include "stdint.h"
#define area 0.00282743338823081391461637904495  //m2
#define resistance 1,838592     //Om
#define coil 112        //piece
void pwmValue(float k, float omega, float induction, int sign, uint8_t *result, int *direction);  //расчет ШИМ