/*
  QMC5883L.h - QMC5883L library
  Copyright (c) 2017 e-Gizmo Mechatronix Central
  Rewritten by Amoree.  All right reserved.
  July 10,2017
*/
#include "I2Cdev.h"
#ifndef QMC5883L_h
#define QMC5883L_h
#endif

#define QMC5883L_ADDR 0x0D //The default I2C address is 0D: 0001101


//Registers Control //0x09

#define Mode_Standby      0        //0b00000000
#define Mode_Continuous   1        //0b00000001

#define ODR_10Hz          1        //0b00000000
#define ODR_50Hz          4        //0b00000100
#define ODR_100Hz         8        //0b00001000
#define ODR_200Hz         12       //0b00001100

#define RNG_2G            0        //0b00000000
#define RNG_8G            16       //0b00010000

#define OSR_512           0        //0b00000000
#define OSR_256           64       //0b01000000
#define OSR_128           128      //0b10000000
#define OSR_64            192      //0b11000000

typedef uint8_t byte;

void QMC5883L_setAddress(uint8_t addr);
void QMC5883L_init();
void QMC5883L_setMode(uint16_t mode,uint16_t odr,uint16_t rng,uint16_t osr);
void QMC5883L_softReset();
void QMC5883L_read(int* result);
void QMC5883L_WriteReg(uint8_t Reg,uint8_t val);




