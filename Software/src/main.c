#include "stm32f10x.h"
#include "uart.h"
#include "MPU6050.h"
#include <math.h>

double _asin(double i) {return asin(i);}
double _atan2(double i,double k) {return atan2(i,k);}
double _sqrt(double i) {return sqrt(i);}


#define NVIC_GROUPING	3

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

int main() {
	NVIC_SetPriorityGrouping(0x07 - NVIC_GROUPING);

	uart_init(72, 115200);
	MPU_init();
	SixAxis imuData;

	while(1) {
		MPU6050_getStructData(&imuData);
		IMU_comput(imuData);

		uart_sendStr("Pitch Angle: ");
		uart_Float2Char(g_Pitch);

		uart_sendStr("; Roll Angle: ");
		uart_Float2Char(g_Roll);

		uart_sendStr("; Yaw Angle: ");
		uart_Float2Char(g_Yaw);

		UART_CR();

		delay_ms(100);
	}
	while(1);
}
