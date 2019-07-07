/*
  QMC5883L.cpp - QMC5883L library
  Copyright (c) 2017 e-Gizmo Mechatronix Central
  Rewritten by Amoree.  All right reserved.
  July 10,2017

  SET continuous measurement mode
  OSR = 512
  Full Scale Range = 8G(Gauss)
  ODR = 200HZ

*/

#include "QMC5883L.h"



/* Записать значение val в регистр */
void QMC5883L_WriteReg(byte Reg, byte val) {
	I2Cdev_writeByte(QMC5883L_ADDR, Reg, val);
}

/* Инициализация магнитометра */
void QMC5883L_init() {
	QMC5883L_WriteReg(0x0B, 0x01);
	// Define Set/Reset period
	byte mode_write = Mode_Continuous +  ODR_200Hz + RNG_8G + OSR_512;
	QMC5883L_WriteReg(0x09,mode_write);
}

/* Сбросить софт (трогать не нужно)*/
void QMC5883L_softReset() {
	QMC5883L_WriteReg(0x0A, 0x80);
}

/* Считать значение с магнитометра в result */
void QMC5883L_read(int* result) {
	int regAddress = 0x00;
	int tempAddress = 0x07;
	byte buff[6];
	//byte tempBuff[2];
	I2Cdev_readBytes(QMC5883L_ADDR, regAddress, 8, buff, 0);
	//I2Cdev_readBytes(QMC5883L_ADDR, tempAddress, 2, tempBuff, 0);
	result[0] = ((buff[0] << 8) | buff[1]);		   // x-direction
	result[1] = ((buff[2] << 8) | buff[3]);		   // y-direction
	result[2] = ((buff[4] << 8) | buff[5]);		   // z-direction
	//result[3] = ((tempBuff[0] << 8) | tempBuff[1]); // temperature
}
