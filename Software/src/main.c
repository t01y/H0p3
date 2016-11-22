#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bit.h"

#define LED0 BIT_ADDR((GPIOA_BASE+12), 8)
#define LED1 BIT_ADDR((GPIOD_BASE+12), 2)
#define LED_TOGGLE(pin) (pin) = ~(pin)

void delay_ms(volatile unsigned int count) {
    for(count *= 12000; count!=0; count--);
}

void Init_led() {
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;       //GPIOA enable
	GPIOA->CRH &= 0xFFFFFFF0;
	GPIOA->CRH |= 0x00000003;

	RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;
	GPIOD->CRL &= 0xFFFFF0FF;
	GPIOD->CRL |= 0x00000300;
}

void LedTask0() {
	while(1) {
		LED0 = 0;
		vTaskDelay(100);
		LED0 = 1;
		vTaskDelay(400);
	}
}

void LedTask1() {
	while(1) {
		LED1 = 0;
		vTaskDelay(100);
		LED1 = 1;
		vTaskDelay(600);
	}
}

int main() {
	// NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	Init_led();

	xTaskCreate(LedTask0, "LED_TASK0", 40, NULL, 1, NULL);
	xTaskCreate(LedTask1, "LED_TASK1", 40, NULL, 2, NULL);
	vTaskStartScheduler();
	while(1);
}
