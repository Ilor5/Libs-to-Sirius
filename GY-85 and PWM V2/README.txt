There are few steps to generate code for B-dot:
1. Cut folders Inc and Src to your own folder;
2. Open GY-85 and PWM V2.ioc and press Generate Code (right top angle);
3. Open EWARM folder and open Project.eww;
4. Click on Applications (left side), then click on User and choose main.c
5. Click Ctrl+F7 to compile the project. 
6. Copy next files to Src folder: calculation.c, I2Cdev.c, ITG3205.c, QMC5883L.cpp, calculation.h, I2Cdev.h, ITG3205.h, QMC5883L.h;
7. In Project.eww Click on Applications (left side), then click on User and right click on it and choose Add - Add Files.
8. Choose added in step 6 files.
9. Open main.c that was cuted in step 1.
10. Copy missing parts from beginning to While loop of code from opened main.c to main.c that located in Src folder 
    