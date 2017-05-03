#include "stm32f10x.h"
#include "uart.h"
#include "oled.h"

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
	// delay_ms(50);
	oled_init();
	ramDrawPoint(0, 0, 2);
	ramDrawPoint(0, 63, 2);
	ramDrawPoint(255, 0, 2);
	ramDrawPoint(255, 63, 2);

	ramDrawPoint(127, 32, 6);

	unsigned int offset = 0;
	while(1) {
		oled_DrawViewPort(offset++, 0);
		delay_ms(10);
	}
	while(1);
}
