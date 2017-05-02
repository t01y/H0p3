#ifndef OLED_H
#define OLED_H

#define SDIN_PIN	6
#define SCLK_PIN	7
#define DC_PIN		0
#define RST_PIN		1
#define CS_PIN		3

#define SDIN	BIT_ADDR(&(GPIOA->ODR), SDIN_PIN)
#define SCLK	BIT_ADDR(&(GPIOA->ODR), SCLK_PIN)
#define DC		BIT_ADDR(&(GPIOB->ODR), DC_PIN)
#define RST		BIT_ADDR(&(GPIOB->ODR), RST_PIN)
#define CS		BIT_ADDR(&(GPIOA->ODR), CS_PIN)

extern void delay_us(unsigned int t);
extern void delay_ms(unsigned int t);

#define OLED_DELAY()	do {\
	delay_us(1);\
} while(0)

void oled_init();
void oled_sendSingleByte(unsigned char, unsigned char);
void fill_ram (unsigned char h, unsigned char l);

#define OLED_CMD_FLAG	0
#define OLED_DATA_FLAG	1

#define oled_cmd(k)		do {\
	oled_sendSingleByte((k), OLED_CMD_FLAG);\
} while(0)

#define oled_data(k)	do {\
	oled_sendSingleByte((k), OLED_DATA_FLAG);\
} while(0)

#endif
