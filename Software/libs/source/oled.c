#include "oled.h"
#include "stm32f10x.h"
#include "bit.h"

unsigned char display_mem[OLED_DISPLAY_MEM_HEIGHT][OLED_DISPLAY_MEM_WIDTH] = {};
const unsigned short TB[] = {
	0x0000,
	0x0821,
	0x1863,
	0x38E7,
	0x79EF,
	0xFBFF,
	0xFFFF
};

unsigned char block1[] = {
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 6, 6, 6, 6, 6, 6, 0,
	0, 6, 2, 2, 2, 2, 6, 0,
	0, 6, 2, 0, 0, 2, 6, 0,
	0, 6, 2, 0, 0, 2, 6, 0,
	0, 6, 2, 0, 0, 2, 6, 0,
	0, 6, 2, 2, 2, 2, 6, 0,
	0, 6, 6, 6, 6, 6, 6, 0
};

void oled_init() {
	// initial I/O Port
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN|RCC_APB2ENR_IOPAEN;

	GPIOA->CRL &= 0x00FF0FFF;	//PA6 PA7 PA3
	GPIOA->CRL |= 0x33003000;

	GPIOB->CRL &= 0xFFFFFF00;	//PB0 PB1
	GPIOB->CRL |= 0x00000033;


	// initial SSD1351
	RST = 1;
	delay_ms(1);
	RST = 0;
	delay_ms(10);
	RST = 1;

	//Command Lock
	oled_cmd(0xFD);
	oled_data(0x12);

	//Command Lock
	oled_cmd(0xFD);
	oled_data(0xB1);

	//Set Sleep Mode On
	oled_cmd(0xAE);

	//Set Display Clock Divide Ratio/Oscillator Frequency
	oled_cmd(0xB3);
	oled_data(0xF1);

	//Set Multiplex Ratio
	oled_cmd(0xCA);
	oled_data(0x7F);

	//Set Display Offset
	oled_cmd(0xA2);
	oled_data(0x00);

	//Set Display Start Line
	oled_cmd(0xA1);
	oled_data(0x00);

	//Set Re-Map & Color Depth
	oled_cmd(0xA0);
	oled_data(0x74);

	//Set GPIO
	oled_cmd(0xB5);
	oled_data(0x00);


	//Function Selection
	oled_cmd(0xAB);
	oled_data(0x01);

	//Set Segment Low Voltage
	oled_cmd(0xB4);
	oled_data(0xA0);
	oled_data(0xB5);
	oled_data(0x55);

	//Set Contrast Current
	oled_cmd(0xC1);
	oled_data(0xC8);
	oled_data(0x80);
	oled_data(0xC8);

	//Set Master Current Control
	oled_cmd(0xC7);
	oled_data(0x0F);

	//Gamma Look up Table
	oled_cmd(0xB8);
	oled_data(0x02);
	oled_data(0x03);
	oled_data(0x04);
	oled_data(0x05);
	oled_data(0x06);
	oled_data(0x07);
	oled_data(0x08);
	oled_data(0x09);
	oled_data(0x0A);
	oled_data(0x0B);
	oled_data(0x0C);
	oled_data(0x0D);
	oled_data(0x0E);
	oled_data(0x0F);
	oled_data(0x10);
	oled_data(0x11);
	oled_data(0x12);
	oled_data(0x13);
	oled_data(0x15);
	oled_data(0x17);
	oled_data(0x19);
	oled_data(0x1B);
	oled_data(0x1D);
	oled_data(0x1F);
	oled_data(0x21);
	oled_data(0x23);
	oled_data(0x25);
	oled_data(0x27);
	oled_data(0x2A);
	oled_data(0x2D);
	oled_data(0x30);
	oled_data(0x33);
	oled_data(0x36);
	oled_data(0x39);
	oled_data(0x3C);
	oled_data(0x3F);
	oled_data(0x42);
	oled_data(0x45);
	oled_data(0x48);
	oled_data(0x4C);
	oled_data(0x50);
	oled_data(0x54);
	oled_data(0x58);
	oled_data(0x5C);
	oled_data(0x60);
	oled_data(0x64);
	oled_data(0x68);
	oled_data(0x6C);
	oled_data(0x70);
	oled_data(0x74);
	oled_data(0x78);
	oled_data(0x7D);
	oled_data(0x82);
	oled_data(0x87);
	oled_data(0x8C);
	oled_data(0x91);
	oled_data(0x96);
	oled_data(0x9B);
	oled_data(0xA0);
	oled_data(0xA5);
	oled_data(0xAA);
	oled_data(0xAF);
	oled_data(0xB4);
	oled_data(0x00);

	//Set Phase Length
	oled_cmd(0xB1);
	oled_data(0x32);


	//Enhance Driving Scheme Capability
	oled_cmd(0xB2);
	oled_data(0xA4);
	oled_data(0x00);
	oled_data(0x00);

	//Set Pre-Charge Voltage
	oled_cmd(0xBB);
	oled_data(0x17);

	//Set Second Pre-Charge Period
	oled_cmd(0xB6);
	oled_data(0x01);

	//Set VCOMH Voltage
	oled_cmd(0xBE);
	oled_data(0x05);

	//Set Display Mode
	oled_cmd(0xA6);

	//Clear Screen
	// fill_ram (0xF0, 0x00);

	//Set Sleep Mode Off
	oled_cmd(0xAF);
	delay_ms(1000);

}

void oled_sendSingleByte(unsigned char c, unsigned char cmdFlag) {
	CS = 0;
	DC = cmdFlag;
	for(char bit = 0; bit < 8; bit++) {
		SCLK = 0;
		SDIN = (c & 1<<(7-bit)) >> (7-bit);
		SCLK = 1;
	}
	SCLK = 0;
	CS = 1;
}

void oled_send2Bytes(unsigned short d) {
	oled_data(((unsigned char *)&d)[0]);
	oled_data(((unsigned char *)&d)[1]);
}

void oled_DrawViewPort(unsigned char x, unsigned char y) {
	oled_cmd(0x15);
	oled_data(0x00);
	oled_data(0x7F);

	oled_cmd(0x75);
	oled_data(0x00);
	oled_data(0x7F);

	oled_cmd(0x5C);
	for(unsigned char i = 0; i < OLED_PIXEL_HEIGHT; i++) {
		for(unsigned char j = 0; j < OLED_PIXEL_WIDTH; j++) {
			// unsigned char *rgb = (unsigned char *)&display_mem[(y+i)&0x7F][(x+j)&0x7F];
			// oled_data(rgb[0]);
			// oled_data(rgb[1]);
			oled_send2Bytes(TB[display_mem[(y+i)&0xFF][(x+j)&0x7F]]);
		}
	}
}



void ramInsertBlock(unsigned char x, unsigned char y, unsigned char* data) {
	for(unsigned char i = 0; i < OLED_BLOCK_HEIGHT; i++) {
		for(unsigned char j = 0; j < OLED_BLOCK_WIDTH; j++) {
			display_mem[y+i][x+j] = *data++;
		}
	}

}
