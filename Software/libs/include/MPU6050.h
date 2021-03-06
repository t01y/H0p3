#ifndef MPU6050_H
#define MPU6050_H

void delay(volatile unsigned int count);
//=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_

#define MPU6050_ADDR 0xD0 //    0x68 >> 1

#define SMPLRT_DIV      0x19
#define CONFIG          0x1A
#define GYRO_CONFIG     0x1B
#define ACCEL_CONFIG    0x1C

#define ACCEL_XOUT_H    0x3B
#define ACCEL_XOUT_L    0x3C
#define ACCEL_YOUT_H    0x3D
#define ACCEL_YOUT_L    0x3E
#define ACCEL_ZOUT_H    0x3F
#define ACCEL_ZOUT_L    0x40

#define TEMP_OUT_H      0x41
#define TEMP_OUT_L      0x42

#define GYRO_XOUT_H     0x43
#define GYRO_XOUT_L     0x44
#define GYRO_YOUT_H     0x45
#define GYRO_YOUT_L     0x46
#define GYRO_ZOUT_H     0x47
#define GYRO_ZOUT_L     0x48

#define PWR_MGMT_1      0x6B
#define WHO_AM_I        0x75

//=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_=_
#define IMU_SOFTWARE_FIXED

#define G_X_OFFSET 1.5853f
#define G_Y_OFFSET 0.7926f
#define G_Z_OFFSET 2.0121f

#define A_X_OFFSET 0
#define A_Y_OFFSET 0
#define A_Z_OFFSET 0.2f

#define IMU_ADDRESS 0x68
#define IMU_NOT_CONNECTED (MPU_Sigle_Read(WHO_AM_I)!=IMU_ADDRESS)

typedef struct{
    float gX;
    float gY;
    float gZ;
    float aX;
    float aY;
    float aZ;
}SixAxis, *pSixAxis;

#define Kp      100.0f      //比例增益支配率(常量)
#define Ki      0.002f      //积分增益支配率
#define halfT   0.001f      //采样周期的一半

float g_Yaw, g_Pitch, g_Roll;

extern double _asin (double);
extern double _atan2 (double,double);
extern double _sqrt (double);


void MPU_Sigle_Write(unsigned char reg_addr, unsigned char reg_data);
void MPU_Write2bytes(unsigned char reg_addr, unsigned short word);
void MPU_writeBytes(unsigned char writeAddr, unsigned char length, unsigned char *data);
void MPU_readBytes(unsigned char readAddr, unsigned char length, unsigned char *data);
unsigned char MPU_Sigle_Read(unsigned reg_addr);
short MPU_GetData(unsigned char REG_Addr);
void MPU_init();
void MPU6050_getStructData(pSixAxis cache);
void MPU6050_debug(pSixAxis cache);
void IMU_Comput(SixAxis);

#endif
