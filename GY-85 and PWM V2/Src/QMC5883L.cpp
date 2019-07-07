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

uint8_t address = QMC5883L_ADDR;

void QMC5883L_setAddress(uint8_t addr) {
	address = addr;
}

void QMC5883L_WriteReg(byte Reg, byte val) {
	I2Cdev_writeByte(address, Reg, val);
}

void QMC5883L_init() {
	QMC5883L_WriteReg(0x0B, 0x01);
	// Define Set/Reset period
	byte mode_write = 1 + 12 + 16;
	QMC5883L_WriteReg(0x09,mode_write);
	//QMC5883L_setMode(Mode_Continuous, ODR_200Hz, RNG_8G, OSR_512);
}

void QMC5883L_setMode(uint16_t mode, uint16_t odr, uint16_t rng, uint16_t osr) {
	QMC5883L_WriteReg(0x09, mode | odr | rng | osr);
	// Serial.println(mode|odr|rng|osr,HEX);
}

void QMC5883L_softReset() {
	QMC5883L_WriteReg(0x0A, 0x80);
}

void QMC5883L_read(int* result) {
	// Wire.beginTransmission(address);
	// Wire.write(0x00);
	// Wire.endTransmission();
	int regAddress = 0x00;
	int tempAddress = 0x07;
	byte buff[6];
	byte tempBuff[2];
	// Wire.requestFrom(address, 6);
	// *x = Wire.read();                           //LSB  x
	// *x |= Wire.read() << 8;                     //MSB  x
	// *y = Wire.read();                           //LSB  z
	// *y |= Wire.read() << 8;                     //MSB z
	// *z = Wire.read();                           //LSB y
	// *z |= Wire.read() << 8;                     //MSB y
	I2Cdev_readBytes(QMC5883L_ADDR, regAddress, 8, buff, 0);
	I2Cdev_readBytes(QMC5883L_ADDR, tempAddress, 2, tempBuff, 0);
	result[0] = ((buff[0] << 8) | buff[1]);		   // x-direction
	result[1] = ((buff[2] << 8) | buff[3]);		   // y-direction
	result[2] = ((buff[4] << 8) | buff[5]);		   // z-direction
	result[3] = ((tempBuff[0] << 8) | tempBuff[1]); // temperature
}
