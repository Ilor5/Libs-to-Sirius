// Либа для магнитометра
#include "QMC5883L.h"


	float a[3][3];
	float b[3];

/* Записать значение val в регистр */
void QMC5883L_WriteReg(byte Reg, byte val) {
	 I2Cdev_writeByte(QMC5883L_ADDR, Reg, val);
}

/* Инициализация магнитометра */
void QMC5883L_init() {
	QMC5883L_WriteReg(0x0B, 0x01);
	QMC5883L_WriteReg(0x09, 0x1D);
  switch (SAT_NUMBER){
    case 1:
      a[0][0] = 1.070921;
       a[0][1] = -0.005472;
       a[0][2] = -0.013059;
       a[1][1] = 1.031901;
       a[1][2] = -0.008048;
       a[2][2] = 1.035044;
       a[1][0] = a[0][1];
       a[2][0] = a[0][2];
       a[2][1] = a[1][2];
       b[0] = -10.683033;
       b[1] = -0.215056;
       b[2] = 7.296592;
       break;
    case 2:
        b[0]= 1.898120;
        b[1]= 1.298545;
        b[2]= 3.829979;
        a[0][0]= 0.992233;
        a[0][1]=-0.000157;
        a[0][2]= 0.044873;
        a[1][1]= 1.072447;
        a[1][2]= -0.023032;
        a[2][2]= 1.039227;
        a[1][0] = a[0][1];
        a[2][0] = a[0][2];
        a[2][1] = a[1][2];
        break;
      case 3:
        a[0][0] = 1.116970;
        a[0][1] = -0.003589;
        a[0][2] = -0.011106;
        a[1][1] = 1.172562;
        a[1][2] = 0.004734;
        a[2][2] = 1.208839;
        a[1][0] = a[0][1];
        a[2][0] = a[0][2];
        a[2][1] = a[1][2];
        b[0] = 6.928587;
        b[1] = 10.660768;
        b[2] = -2.674679;
        break;
      default:
        break;
  } 
}

/* Сбросить софт (трогать не нужно)*/
void QMC5883L_softReset() {
	QMC5883L_WriteReg(0x0A, 0x80);
}

/* Считать значение с магнитометра в result */
void QMC5883L_read(float *result) {
	int regAddress = 0x00; //адрес регистра (не трогать)
	byte buff[6];	//буфер для считывания данных
	float dataMagnet[3];  // массив данных магнитометра
	float rawDataMagnet[3];	// массив сырых данных магнитометра
	I2Cdev_readBytes(QMC5883L_ADDR, regAddress, 6, buff, 0);

	rawDataMagnet[0] = ((buff[1] << 8) + buff[0]); // x-direction
	rawDataMagnet[1] = ((buff[3] << 8) + buff[2]); // y-direction
	rawDataMagnet[2] = ((buff[5] << 8) + buff[4]); // z-direction
	
	for (int i = 0; i < 3; i++) {
		if (rawDataMagnet[i] > 32768) {
			rawDataMagnet[i] = rawDataMagnet[i] - 65536;		//проверка на переполнение
		}
	}

	//заполнение массива данных
	dataMagnet[0] = rawDataMagnet[0] / 30.0;
	dataMagnet[1] = rawDataMagnet[1] / 30.0;
	dataMagnet[2] = rawDataMagnet[2] / 30.0;      
        
	
	//калибровочные параметры (выставить в зависимости от спутника)
	float Bx, By, Bz;  

	//вывод калиброванных данных магнитометра
	result[0] = a[0][0] * (dataMagnet[0] - b[0]) + a[0][1] * (dataMagnet[1] - b[1]) + a[0][2] * (dataMagnet[2] - b[2]);
	result[1] = a[1][0] * (dataMagnet[0] - b[0]) + a[1][1] * (dataMagnet[1] - b[1]) + a[1][2] * (dataMagnet[2] - b[2]);
	result[2] = a[2][0] * (dataMagnet[0] - b[0]) + a[2][1] * (dataMagnet[1] - b[1]) + a[2][2] * (dataMagnet[2] - b[2]);
//        result[0] = dataMagnet[0];
  //              result[1] = dataMagnet[1];
    //                    result[2] = dataMagnet[2];
        
}
