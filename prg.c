#include<stdio.h> 
#include<fcntl.h> 
#include<sys/ioctl.h> 
#include<linux/i2c.h> 
#include<linux/i2c-dev.h> 
#include <signal.h>
#include <math.h>
//#include <chrono>
//#include "MahonyAHRS/MahonyAHRS.h"

#define DEVID 0x00 
#define BUFFER_SIZE 40 
#define OUT_X_L 0x28 
#define OUT_X_H 0x29 
#define OUT_Y_L 0x2A 
#define OUT_Y_H 0x2B 
#define OUT_Z_L 0x2C 
#define OUT_Z_H 0x2D 
#define OUT_X_L_M 0x08 
#define OUT_X_H_M 0x09 
#define OUT_Y_L_M 0x0A 
#define OUT_Y_H_M 0x0B 
#define OUT_Z_L_M 0x0C 
#define OUT_Z_H_M 0x0D

//extern volatile float q0, q1, q2, q3; 
FILE *myoutput = NULL;
int finished = 0;
int main_counter = 0;


int binTwosComplementToSignedDecimal(int mynum,int significantBits) 
{
    char binary[9];
    int power = pow(2,significantBits-1);
    int sum = 0;
    int i, j;
    binary[8] = 0;

    for (i = 0; i < 8; ++i) {
	if (((mynum >> i) & 1) == 0) {
		binary[7-i] = '0';
	} else {
		binary[7-i] = '1';
	}
    }

    //printf("%s vs %d\n", binary, mynum);

    for (i=0; i<significantBits; ++i)
    {
        if ( i==0 && binary[i]!='0')
        {
            sum = power * -1;
        }
        else 
        {
            sum += (binary[i]-'0')*power;//The -0 is needed
        }
        power /= 2;
    }

    return sum;
}

void close_handle(int s) {
	finished = 1;
}

char readFromAddress(int file, int addr); 
void writeSomeData(int file, char *buff, int n) {
 if (write(file, buff, n) != n) {
	perror("Failed to write\n");
 }
}

int getintval(char xl, char xh) {
	return (binTwosComplementToSignedDecimal(xh, 8) << 8) | binTwosComplementToSignedDecimal(xl, 8);
}
void readAccelMagn(int file, int *gyrX, int *gyrY, int *gyrZ, int *magX, int *magY, int *magZ) {
        if(ioctl(file, I2C_SLAVE, 0x1D) < 0){
                perror("Failed to connect to the sensor\n");
        }
        char xl = readFromAddress(file, OUT_X_L);
        char xh = readFromAddress(file, OUT_X_H);
        *gyrX = getintval(xl, xh);
        xl = readFromAddress(file, OUT_Y_L);
        xh = readFromAddress(file, OUT_Y_H);
        *gyrY = getintval(xl, xh);
        xl = readFromAddress(file, OUT_Z_L);
        xh = readFromAddress(file, OUT_Z_H);
        *gyrZ = getintval(xl, xh);
        //printf("accel (x, y, z): (%d, %d, %d)\n", gyrX, gyrY, gyrZ);
	xl = readFromAddress(file, OUT_X_L_M);
        xh = readFromAddress(file, OUT_X_H_M);
        *magX = getintval(xl, xh);
        xl = readFromAddress(file, OUT_Y_L_M);
        xh = readFromAddress(file, OUT_Y_H_M);
        *magY = getintval(xl, xh);
        xl = readFromAddress(file, OUT_Z_L_M);
        xh = readFromAddress(file, OUT_Z_H_M);
        *magZ = getintval(xl, xh);
        //printf("magn (x, y, z): (%d, %d, %d)\n", gyrX, gyrY, gyrZ);
}
void readGyro(int file, int *gyrX, int *gyrY, int *gyrZ) {
        if(ioctl(file, I2C_SLAVE, 0x6B) < 0){
                perror("Failed to connect to the sensor\n");
        }
        char xl = readFromAddress(file, OUT_X_L);
        char xh = readFromAddress(file, OUT_X_H);
      	*gyrX = getintval(xl, xh);
        xl = readFromAddress(file, OUT_Y_L);
        xh = readFromAddress(file, OUT_Y_H);
        *gyrY = getintval(xl, xh);
        xl = readFromAddress(file, OUT_Z_L);
        xh = readFromAddress(file, OUT_Z_H);
        *gyrZ = getintval(xl, xh);
        //printf("gyro (x, y, z): (%d, %d, %d)\n", gyrX, gyrY, gyrZ);
}
void setupGyro(int file) {
   if(ioctl(file, I2C_SLAVE, 0x6B) < 0){
        perror("Failed to connect to the sensor\n");
   }
   char writeBuffer[2] = {0x20, 0x8F};
   writeSomeData(file, writeBuffer, 2);
   writeBuffer[0] = 0x21;
   writeBuffer[1] = 0x20;
   writeSomeData(file, writeBuffer, 2);
}
void setupAxelMagn(int file) {
   if(ioctl(file, I2C_SLAVE, 0x1D) < 0){
        perror("Failed to connect to the sensor\n");
   }
   char writeBuffer[2] = {0x20, 0x8F};
   writeSomeData(file, writeBuffer, 2);
   writeBuffer[0] = 0x21;
   writeBuffer[1] = 0x18;
   writeSomeData(file, writeBuffer, 2);
   writeBuffer[0] = 0x24;
   writeBuffer[1] = 0x64;
   writeSomeData(file, writeBuffer, 2);
   writeBuffer[0] = 0x25;
   writeBuffer[1] = 0x20;
   writeSomeData(file, writeBuffer, 2);
   writeBuffer[0] = 0x26;
   writeBuffer[1] = 0x00;
   writeSomeData(file, writeBuffer, 2);
}
int main(){
   int file;
   signal(SIGINT, close_handle);
   myoutput = fopen("output.txt", "w");
   fprintf(myoutput, "accx, accy, accz, magx, magy, magz, gyrx, gyry, gyrz");
   if((file=open("/dev/i2c-2", O_RDWR)) < 0){
      perror("failed to open the bus\n");
      return 1;
   }
   // setup
   printf("setting up gyro\n");
   setupGyro(file);
   printf("setting up accel\n");
   setupAxelMagn(file);
   int counter = 0;
   int gyrx = 0, gyry = 0, gyrz = 0, accx = 0, accy = 0, accz = 0, magx = 0, magy = 0, magz = 0;
   int lastAx = 0, lastAy = 0, lastAz = 0, lastmx = 0, lastmy = 0, lastmz = 0;
   while (finished == 0) {
	readGyro(file, &gyrx, &gyry, &gyrz);
	readAccelMagn(file, &accx, &accy, &accz, &magx, &magy, &magz);
	
	//MahonyAHRSupdate((float)gyrx, (float)gyry, (float)gyrz, (float)accx, (float)accy, (float)accz, (float)magx, (float)magy, (float)magz);
	if ((lastAx != accx || lastAy != accy || lastAz != accz) || (lastmx != magx || lastmy != magy || lastmz != magz)) {
	  fprintf(myoutput, "%d, %d, %d, %d, %d ,%d, %d, %d, %d\n", accx, accy, accz, magx, magy, magx, gyrx, gyry, gyrz);
	  //printf("%d, %d, %d, %d, %d ,%d, %d, %d, %d\n", accx, accy, accz, magx, magy, magx, gyrx, gyry, gyrz);
	  ++main_counter;
	  lastAx = accx;
	  lastAy = accy;
	  lastAz = accz;
	  lastmx = magx;
	  lastmy = magy;
	  lastmz = magz;
	}
   }
   close(file);
   fclose(myoutput);
   printf("total counts %d\n", main_counter);
   return 0;
}
char readFromAddress(int file, int addr) {
   char writeBuffer = addr;
   if(write(file, &writeBuffer, 1)!=1){
     perror("Failed to reset the read address\n");
     return 1;
   }
   char readBuffer[1];
   if(read(file, readBuffer, 1)!=1){
      perror("Failed to read in the buffer\n");
      return 1;
   }
   return readBuffer[0];
}

