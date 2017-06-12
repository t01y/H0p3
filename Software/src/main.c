#include "stm32f10x.h"
#include "i2c.h"
#include <math.h>
#include "uart.h"
// #include "oled.h"
#include "dmp.h"

#define NVIC_GROUPING	3

#define ISP_ADDR		0x1FFFF000

double _asin(double i) {return asin(i);}
double _atan2(double i,double k) {return atan2(i,k);}
// double _sqrt(double i) {return sqrt(i);}

void delay_ms(unsigned int t) {
	SysTick->LOAD = 9000 * t;
	SysTick->VAL = 0;
	SysTick->CTRL = 0x01;
	for(unsigned int tmp = SysTick->CTRL;(tmp&0x01)&&(!(tmp&SysTick_CTRL_COUNTFLAG));tmp = SysTick->CTRL);
	SysTick->CTRL = 0;
	SysTick->VAL = 0;
}

// 延时函数, 单位为微秒
void delay_us(unsigned int t) {
	SysTick->LOAD = 9 * t;
	SysTick->VAL = 0;
	SysTick->CTRL = 0x01;
	for(unsigned int tmp = SysTick->CTRL;(tmp&0x01)&&(!(tmp&SysTick_CTRL_COUNTFLAG));tmp = SysTick->CTRL);
	SysTick->CTRL = 0;
	SysTick->VAL = 0;
}

void jump2ISP() {
	__set_MSP(*(unsigned int *)ISP_ADDR);
	((void (*)(void))*((unsigned int *)(ISP_ADDR + 4)))();
}


Quaternion q;           // [w, x, y, z]         quaternion container
float euler[3];         // [psi, theta, phi]    Euler angle container
#define	M_PI 3.14f


int main() {
	NVIC_SetPriorityGrouping(0x07 - NVIC_GROUPING);

	uart_init(72, 115200);
	IIC_init();

	delay_ms(1000);	// Delay is required after MPU6050 powered up, At least 7ms
	// MPU_init();
	MPUinitialize();

	DMP_Initialize();
	MPUsetDMPEnabled(true);

	unsigned char fifoBuffer[128];

	unsigned char mpuIntStatus;
	unsigned char fifoCount;

	while(1) {
		mpuIntStatus = MPUgetIntStatus();
	    fifoCount = MPUgetFIFOCount();
		if((mpuIntStatus & 0x10) || fifoCount == 1024) {
			MPUresetFIFO();
	        uart_sendStr("\r\nFIFO overflow!");
		} else if(mpuIntStatus & 0x02) {
			while (fifoCount < 42)
				fifoCount = MPUgetFIFOCount();
			MPUgetFIFOBytes(fifoBuffer, 42);
			fifoCount -= 42;

			MPUdmpGetQuaternion(&q, fifoBuffer);
			MPUdmpGetEuler(euler, &q);

			UART_CR();
			uart_Float2Char(euler[0]);
			uart_sendStr(", ");
			uart_Float2Char(euler[1]);
			uart_sendStr(", ");
			uart_Float2Char(euler[2]);
			uart_sendStr(", ");
			


		}


	}
	while(1);
}
