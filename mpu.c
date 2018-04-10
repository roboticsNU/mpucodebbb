#include <unistd.h>
#include <time.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <signal.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <chrono>

using namespace std;
using namespace std::chrono;

#define DEVID 0x68
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

#define GYRO_XOUT_H  0x43
#define GYRO_XOUT_L  0x44
#define GYRO_YOUT_H  0x45
#define GYRO_YOUT_L  0x46
#define GYRO_ZOUT_H  0x47
#define GYRO_ZOUT_L  0x48

int file = 0;
int volatile finished = 0;
int main_counter = 0;
char readingDataSetup[6];

char accelXH = ACCEL_XOUT_H;
char accelXL = ACCEL_XOUT_L;
char accelYH = ACCEL_YOUT_H;
char accelYL = ACCEL_YOUT_L;
char accelZH = ACCEL_ZOUT_H;
char accelZL = ACCEL_ZOUT_L;

char gyroXH = GYRO_XOUT_H;
char gyroXL = GYRO_XOUT_L;
char gyroYH = GYRO_YOUT_H;
char gyroYL = GYRO_YOUT_L;
char gyroZH = GYRO_ZOUT_H;
char gyroZL = GYRO_ZOUT_L;

void close_handle(int s) {
	finished = 1;
}

void setupMpu() {
	char data[255];
	printf("setting up 0x68 addr\n");
	if(ioctl(file, I2C_SLAVE, 0x68) < 0){
                perror("Failed to connect to the sensor\n");
        }
	data[0] = 0x6b;
	data[1] = 0x03;
	printf("writing data for setting up mpu\n");
	write(file, data, 2);
	data[0] = 0x38;
	data[1] = 1;
	write(file, data, 2);
	data[0] = 0x23;
        data[1] = 0x00;
        write(file, data, 2);
	data[0] = 0x6A;
        data[1] = 0x00;
        write(file, data, 2);
	data[0] = 0x24;
	data[1] = 0x0D;
        write(file, data, 2);
	data[0] = 0x1B;
	data[1] = 0x10;
        write(file, data, 2);
}


int readAccel2(int16_t *accx, int16_t *accy, int16_t *accz) {
	char data[9];
	char from = 0x64;
	char fch = 0x72, fcl = 0x73;
	char countl = 0, counth = 0;
	//char xh, xl, yh, yl, zh, zl;
	write(file, &fcl, 1);
	read(file, &countl, 1);
	write(file, &fch, 1);
	read(file, &counth, 1);
	if (((counth << 8) | countl) > 0) {
		int value = ((counth << 8) | countl);
		cout << "fifo counter is " << value << endl;
		write(file, &from, 1);
		read(file, data, 9);

		*accx = ((int16_t)data[1] << 8) | data[0];
		*accy = ((int16_t)data[3] << 8) | data[2];
		*accz = ((int16_t)data[5] << 8) | data[4];
		return 1;
	} else {
		return 0;
	}
}

int readAccel(int16_t *accx, int16_t *accy, int16_t *accz) {
	char xh, xl, yh, yl, zh, zl;

	write(file, &accelXL, 1);
	read(file, &xl, 1);
	write(file, &accelXH, 1);
	read(file, &xh, 1);

	write(file, &accelYL, 1);
	read(file, &yl, 1);
	write(file, &accelYH, 1);
	read(file, &yh, 1);

	write(file, &accelZL, 1);
	read(file, &zl, 1);
	write(file, &accelZH, 1);
	read(file, &zh, 1);

	*accx = ((int16_t)xh << 8) | xl;
	*accy = ((int16_t)yh << 8) | yl;
	*accz = ((int16_t)zh << 8) | zl;
	return 1;
}

int readGyro(int16_t *x, int16_t *y, int16_t *z) {
        char xh, xl, yh, yl, zh, zl;

        write(file, &gyroXL, 1);
        read(file, &xl, 1);
        write(file, &gyroXH, 1);
        read(file, &xh, 1);

        write(file, &gyroYL, 1);
        read(file, &yl, 1);
        write(file, &gyroYH, 1);
        read(file, &yh, 1);

        write(file, &gyroZL, 1);
        read(file, &zl, 1);
        write(file, &gyroZH, 1);
        read(file, &zh, 1);

        *x = ((int16_t)xh << 8) | xl;
        *y = ((int16_t)yh << 8) | yl;
        *z = ((int16_t)zh << 8) | zl;
        return 1;
}

int main(int argc, char **argv) {
   readingDataSetup[0] = ACCEL_XOUT_L;
   readingDataSetup[1] = ACCEL_XOUT_H;
   readingDataSetup[2] = ACCEL_YOUT_L;
   readingDataSetup[3] = ACCEL_YOUT_H;
   readingDataSetup[4] = ACCEL_ZOUT_L;
   readingDataSetup[5] = ACCEL_ZOUT_H;

   signal(SIGINT, close_handle);
   char myfilename[255];
   sprintf(myfilename, "output%s_%s.txt", argv[1], argv[2]);
   cout << myfilename << endl;
   ofstream myoutput;
   myoutput.open(myfilename);
   
   myoutput <<  "calib data acc, gyr: 0, 0, 0, 0, 0, 0\n";
   if((file=open("/dev/i2c-1", O_RDWR)) < 0){
      perror("failed to open the bus\n");
      return 1;
   } else {
     printf("bus opened\n");
   }
   // setup
   setupMpu();
   int counter = 0;
   int16_t gyrx = 0, gyry = 0, gyrz = 0, accx = 0, accy = 0, accz = 0, magx = 0, magy = 0, magz = 0;
   int16_t lastAx = 0, lastAy = 0, lastAz = 0, lastmx = 0, lastmy = 0, lastmz = 0;
   double timeSpentTotal = 0;
   double globalTime = 0;
   high_resolution_clock::time_point t1 = high_resolution_clock::now();
   unsigned int globalDuration1 = 0;
   while (finished == 0) {
	high_resolution_clock::time_point tt1 = high_resolution_clock::now();
	readAccel(&accx, &accy, &accz);
	readGyro(&gyrx, &gyry, &gyrz);
	high_resolution_clock::time_point tt2 = high_resolution_clock::now();
	
	auto duration1 = duration_cast<microseconds>( tt2 - tt1 ).count();
	myoutput << accx << "\t" << accy << "\t" << accz <<  "\t" << gyrx << "\t" << gyry << "\t" << gyrz << "\t" <<  duration1 << "\t" <<  duration1 << "\n";
	main_counter++;
	globalDuration1 += duration1;
   }

   high_resolution_clock::time_point t2 = high_resolution_clock::now();
   auto duration = duration_cast<microseconds>( t2 - t1 ).count();

   close(file);
   myoutput.close();
   
   cout << "total counts " << main_counter << " average time on reading " << endl;
//   printf("total counts %d, time average loop iteration cycle %d milliseconds\n", main_counter, globalDuration/main_counter);
   cout << "average time loop iteration is " << (double)globalDuration1 / (double)main_counter << " milliseconds " << endl;
   cout << "Total time spent " << duration << " microseconds" << endl;
   cout << "frames per second: " << main_counter / (duration / (1000 * 1000)) << endl;
   return 0;
}
