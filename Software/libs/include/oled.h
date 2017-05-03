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


void oled_init();
void oled_sendSingleByte(unsigned char, unsigned char);
void oled_DrawViewPort(unsigned char h, unsigned char l);
void ramInsertBlock(unsigned char x, unsigned char y, unsigned char* data);
// (16 - 11) + (8 - 5); (8 - 5) + (8 - 6); 0 + (8 - 5)
#define RGB(x)	(unsigned short)((((x)>>8 )& 0xF800) | (((x)>>5) & 0x07E0) | (((x)>>3) & 0x001F))

#define OLED_CMD_FLAG	0
#define OLED_DATA_FLAG	1

#define oled_cmd(k)		do {\
	oled_sendSingleByte((k), OLED_CMD_FLAG);\
} while(0)

#define oled_data(k)	do {\
	oled_sendSingleByte((k), OLED_DATA_FLAG);\
} while(0)

#define oled_send2Bytes(d) do {\
	oled_data(((unsigned char *)&(d))[0]);\
	oled_data(((unsigned char *)&(d))[1]);\
} while(0)

#define ramInsertPoint(x, y, c) do {\
	display_mem[(unsigned char)(y)][(unsigned char)(x)] = (unsigned char)(c);\
} while(0)

// GUI
#define OLED_COLOR_BYTE				2

#define OLED_BLOCK_WIDTH			8
#define OLED_BLOCK_HEIGHT			8
#define OLED_PIXEL_WIDTH			128
#define OLED_PIXEL_HEIGHT			128
#define OLED_DISPLAY_MEM_WIDTH		128
#define OLED_DISPLAY_MEM_HEIGHT		256


// unsigned short display_mem[OLED_DISPLAY_MEM_HEIGHT][OLED_DISPLAY_MEM_WIDTH];
unsigned char block1[OLED_BLOCK_HEIGHT*OLED_BLOCK_WIDTH];




#endif
