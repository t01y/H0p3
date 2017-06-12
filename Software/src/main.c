#include "stm32f10x.h"
#include <math.h>
#include "uart.h"
// #include "oled.h"
#include "MPU6050.h"
#include "dmp.h"

#define NVIC_GROUPING	3

#define ISP_ADDR		0x1FFFF000

double _asin(double i) {return asin(i);}
double _atan2(double i,double k) {return atan2(i,k);}
double _sqrt(double i) {return sqrt(i);}

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

int main() {
	NVIC_SetPriorityGrouping(0x07 - NVIC_GROUPING);

	uart_init(72, 115200);

	delay_ms(7);	// Delay is required after MPU6050 powered up, At least 7ms
	MPU_init();
	DMP_Initialize();


	while(1) {
	}
	while(1);
}
