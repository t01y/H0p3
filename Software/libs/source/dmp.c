#include "dmp.h"

#include "MPU6050.h"
#include "uart.h"

// Portable
#define I2C_readByte(addr)			MPU_Sigle_Read(addr)
#define I2C_writeByte(addr, data)	MPU_Sigle_Write(addr, data)
#define I2C_writeBytes				MPU_writeBytes
#define I2C_writeWord(addr, data)	MPU_Write2bytes(addr, data)
#define delay_ms					delay_ms
#define print						uart_sendStr
#define printChar(x)				uart_int2char((unsigned int)x)

/* Global variables, will see how to get rid of them
*
*/
// uint8_t MPUdevAddr;
unsigned char MPUbuffer[14];

unsigned short MPUfifoCount;     	// count of all bytes currently in FIFO
// uint8_t  MPUfifoBuffer[64];	// FIFO storage buffer


/* ================================================================================================ *
 | Default MotionApps v2.0 42-byte FIFO packet structure:                                           |
 |                                                                                                  |
 | [QUAT W][      ][QUAT X][      ][QUAT Y][      ][QUAT Z][      ][GYRO X][      ][GYRO Y][      ] |
 |   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  |
 |                                                                                                  |
 | [GYRO Z][      ][ACC X ][      ][ACC Y ][      ][ACC Z ][      ][      ]                         |
 |  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  40  41                          |
 * ================================================================================================ */

// this block of memory gets written to the MPU on start-up, and it seems
// to be volatile memory, so it has to be done each time (it only takes ~1
// second though)
const unsigned char dmpMemory[MPU6050_DMP_CODE_SIZE] = {
    // bank 0, 256 bytes
    0xFB, 0x00, 0x00, 0x3E, 0x00, 0x0B, 0x00, 0x36, 0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00,
    0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0xFA, 0x80, 0x00, 0x0B, 0x12, 0x82, 0x00, 0x01,
    0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x28, 0x00, 0x00, 0xFF, 0xFF, 0x45, 0x81, 0xFF, 0xFF, 0xFA, 0x72, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x7F, 0xFF, 0xFF, 0xFE, 0x80, 0x01,
    0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x3E, 0x03, 0x30, 0x40, 0x00, 0x00, 0x00, 0x02, 0xCA, 0xE3, 0x09, 0x3E, 0x80, 0x00, 0x00,
    0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
    0x41, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x0B, 0x2A, 0x00, 0x00, 0x16, 0x55, 0x00, 0x00, 0x21, 0x82,
    0xFD, 0x87, 0x26, 0x50, 0xFD, 0x80, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x05, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x6F, 0x00, 0x02, 0x65, 0x32, 0x00, 0x00, 0x5E, 0xC0,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xFB, 0x8C, 0x6F, 0x5D, 0xFD, 0x5D, 0x08, 0xD9, 0x00, 0x7C, 0x73, 0x3B, 0x00, 0x6C, 0x12, 0xCC,
    0x32, 0x00, 0x13, 0x9D, 0x32, 0x00, 0xD0, 0xD6, 0x32, 0x00, 0x08, 0x00, 0x40, 0x00, 0x01, 0xF4,
    0xFF, 0xE6, 0x80, 0x79, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD0, 0xD6, 0x00, 0x00, 0x27, 0x10,

    // bank 1, 256 bytes
    0xFB, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xFA, 0x36, 0xFF, 0xBC, 0x30, 0x8E, 0x00, 0x05, 0xFB, 0xF0, 0xFF, 0xD9, 0x5B, 0xC8,
    0xFF, 0xD0, 0x9A, 0xBE, 0x00, 0x00, 0x10, 0xA9, 0xFF, 0xF4, 0x1E, 0xB2, 0x00, 0xCE, 0xBB, 0xF7,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x02, 0x00, 0x02, 0x02, 0x00, 0x00, 0x0C,
    0xFF, 0xC2, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0xCF, 0x80, 0x00, 0x40, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x14,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x03, 0x3F, 0x68, 0xB6, 0x79, 0x35, 0x28, 0xBC, 0xC6, 0x7E, 0xD1, 0x6C,
    0x80, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB2, 0x6A, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xF0, 0x00, 0x00, 0x00, 0x30,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x25, 0x4D, 0x00, 0x2F, 0x70, 0x6D, 0x00, 0x00, 0x05, 0xAE, 0x00, 0x0C, 0x02, 0xD0,

    // bank 2, 256 bytes
    0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x00, 0x00, 0x44, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x01, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x00, 0x00, 0x54, 0x00, 0x00, 0xFF, 0xEF, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00,
    0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    // bank 3, 256 bytes
    0xD8, 0xDC, 0xBA, 0xA2, 0xF1, 0xDE, 0xB2, 0xB8, 0xB4, 0xA8, 0x81, 0x91, 0xF7, 0x4A, 0x90, 0x7F,
    0x91, 0x6A, 0xF3, 0xF9, 0xDB, 0xA8, 0xF9, 0xB0, 0xBA, 0xA0, 0x80, 0xF2, 0xCE, 0x81, 0xF3, 0xC2,
    0xF1, 0xC1, 0xF2, 0xC3, 0xF3, 0xCC, 0xA2, 0xB2, 0x80, 0xF1, 0xC6, 0xD8, 0x80, 0xBA, 0xA7, 0xDF,
    0xDF, 0xDF, 0xF2, 0xA7, 0xC3, 0xCB, 0xC5, 0xB6, 0xF0, 0x87, 0xA2, 0x94, 0x24, 0x48, 0x70, 0x3C,
    0x95, 0x40, 0x68, 0x34, 0x58, 0x9B, 0x78, 0xA2, 0xF1, 0x83, 0x92, 0x2D, 0x55, 0x7D, 0xD8, 0xB1,
    0xB4, 0xB8, 0xA1, 0xD0, 0x91, 0x80, 0xF2, 0x70, 0xF3, 0x70, 0xF2, 0x7C, 0x80, 0xA8, 0xF1, 0x01,
    0xB0, 0x98, 0x87, 0xD9, 0x43, 0xD8, 0x86, 0xC9, 0x88, 0xBA, 0xA1, 0xF2, 0x0E, 0xB8, 0x97, 0x80,
    0xF1, 0xA9, 0xDF, 0xDF, 0xDF, 0xAA, 0xDF, 0xDF, 0xDF, 0xF2, 0xAA, 0xC5, 0xCD, 0xC7, 0xA9, 0x0C,
    0xC9, 0x2C, 0x97, 0x97, 0x97, 0x97, 0xF1, 0xA9, 0x89, 0x26, 0x46, 0x66, 0xB0, 0xB4, 0xBA, 0x80,
    0xAC, 0xDE, 0xF2, 0xCA, 0xF1, 0xB2, 0x8C, 0x02, 0xA9, 0xB6, 0x98, 0x00, 0x89, 0x0E, 0x16, 0x1E,
    0xB8, 0xA9, 0xB4, 0x99, 0x2C, 0x54, 0x7C, 0xB0, 0x8A, 0xA8, 0x96, 0x36, 0x56, 0x76, 0xF1, 0xB9,
    0xAF, 0xB4, 0xB0, 0x83, 0xC0, 0xB8, 0xA8, 0x97, 0x11, 0xB1, 0x8F, 0x98, 0xB9, 0xAF, 0xF0, 0x24,
    0x08, 0x44, 0x10, 0x64, 0x18, 0xF1, 0xA3, 0x29, 0x55, 0x7D, 0xAF, 0x83, 0xB5, 0x93, 0xAF, 0xF0,
    0x00, 0x28, 0x50, 0xF1, 0xA3, 0x86, 0x9F, 0x61, 0xA6, 0xDA, 0xDE, 0xDF, 0xD9, 0xFA, 0xA3, 0x86,
    0x96, 0xDB, 0x31, 0xA6, 0xD9, 0xF8, 0xDF, 0xBA, 0xA6, 0x8F, 0xC2, 0xC5, 0xC7, 0xB2, 0x8C, 0xC1,
    0xB8, 0xA2, 0xDF, 0xDF, 0xDF, 0xA3, 0xDF, 0xDF, 0xDF, 0xD8, 0xD8, 0xF1, 0xB8, 0xA8, 0xB2, 0x86,

    // bank 4, 256 bytes
    0xB4, 0x98, 0x0D, 0x35, 0x5D, 0xB8, 0xAA, 0x98, 0xB0, 0x87, 0x2D, 0x35, 0x3D, 0xB2, 0xB6, 0xBA,
    0xAF, 0x8C, 0x96, 0x19, 0x8F, 0x9F, 0xA7, 0x0E, 0x16, 0x1E, 0xB4, 0x9A, 0xB8, 0xAA, 0x87, 0x2C,
    0x54, 0x7C, 0xB9, 0xA3, 0xDE, 0xDF, 0xDF, 0xA3, 0xB1, 0x80, 0xF2, 0xC4, 0xCD, 0xC9, 0xF1, 0xB8,
    0xA9, 0xB4, 0x99, 0x83, 0x0D, 0x35, 0x5D, 0x89, 0xB9, 0xA3, 0x2D, 0x55, 0x7D, 0xB5, 0x93, 0xA3,
    0x0E, 0x16, 0x1E, 0xA9, 0x2C, 0x54, 0x7C, 0xB8, 0xB4, 0xB0, 0xF1, 0x97, 0x83, 0xA8, 0x11, 0x84,
    0xA5, 0x09, 0x98, 0xA3, 0x83, 0xF0, 0xDA, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xD8, 0xF1, 0xA5,
    0x29, 0x55, 0x7D, 0xA5, 0x85, 0x95, 0x02, 0x1A, 0x2E, 0x3A, 0x56, 0x5A, 0x40, 0x48, 0xF9, 0xF3,
    0xA3, 0xD9, 0xF8, 0xF0, 0x98, 0x83, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0x97, 0x82, 0xA8, 0xF1,
    0x11, 0xF0, 0x98, 0xA2, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xDA, 0xF3, 0xDE, 0xD8, 0x83, 0xA5,
    0x94, 0x01, 0xD9, 0xA3, 0x02, 0xF1, 0xA2, 0xC3, 0xC5, 0xC7, 0xD8, 0xF1, 0x84, 0x92, 0xA2, 0x4D,
    0xDA, 0x2A, 0xD8, 0x48, 0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32, 0xD8, 0x50, 0x71, 0xD9,
    0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58, 0x79, 0xD9, 0x3A, 0xD8, 0x78, 0x93, 0xA3, 0x4D,
    0xDA, 0x2A, 0xD8, 0x48, 0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32, 0xD8, 0x50, 0x71, 0xD9,
    0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58, 0x79, 0xD9, 0x3A, 0xD8, 0x78, 0xA8, 0x8A, 0x9A,
    0xF0, 0x28, 0x50, 0x78, 0x9E, 0xF3, 0x88, 0x18, 0xF1, 0x9F, 0x1D, 0x98, 0xA8, 0xD9, 0x08, 0xD8,
    0xC8, 0x9F, 0x12, 0x9E, 0xF3, 0x15, 0xA8, 0xDA, 0x12, 0x10, 0xD8, 0xF1, 0xAF, 0xC8, 0x97, 0x87,

    // bank 5, 256 bytes
    0x34, 0xB5, 0xB9, 0x94, 0xA4, 0x21, 0xF3, 0xD9, 0x22, 0xD8, 0xF2, 0x2D, 0xF3, 0xD9, 0x2A, 0xD8,
    0xF2, 0x35, 0xF3, 0xD9, 0x32, 0xD8, 0x81, 0xA4, 0x60, 0x60, 0x61, 0xD9, 0x61, 0xD8, 0x6C, 0x68,
    0x69, 0xD9, 0x69, 0xD8, 0x74, 0x70, 0x71, 0xD9, 0x71, 0xD8, 0xB1, 0xA3, 0x84, 0x19, 0x3D, 0x5D,
    0xA3, 0x83, 0x1A, 0x3E, 0x5E, 0x93, 0x10, 0x30, 0x81, 0x10, 0x11, 0xB8, 0xB0, 0xAF, 0x8F, 0x94,
    0xF2, 0xDA, 0x3E, 0xD8, 0xB4, 0x9A, 0xA8, 0x87, 0x29, 0xDA, 0xF8, 0xD8, 0x87, 0x9A, 0x35, 0xDA,
    0xF8, 0xD8, 0x87, 0x9A, 0x3D, 0xDA, 0xF8, 0xD8, 0xB1, 0xB9, 0xA4, 0x98, 0x85, 0x02, 0x2E, 0x56,
    0xA5, 0x81, 0x00, 0x0C, 0x14, 0xA3, 0x97, 0xB0, 0x8A, 0xF1, 0x2D, 0xD9, 0x28, 0xD8, 0x4D, 0xD9,
    0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1, 0x84, 0x0D, 0xDA, 0x0E, 0xD8, 0xA3, 0x29, 0x83, 0xDA,
    0x2C, 0x0E, 0xD8, 0xA3, 0x84, 0x49, 0x83, 0xDA, 0x2C, 0x4C, 0x0E, 0xD8, 0xB8, 0xB0, 0xA8, 0x8A,
    0x9A, 0xF5, 0x20, 0xAA, 0xDA, 0xDF, 0xD8, 0xA8, 0x40, 0xAA, 0xD0, 0xDA, 0xDE, 0xD8, 0xA8, 0x60,
    0xAA, 0xDA, 0xD0, 0xDF, 0xD8, 0xF1, 0x97, 0x86, 0xA8, 0x31, 0x9B, 0x06, 0x99, 0x07, 0xAB, 0x97,
    0x28, 0x88, 0x9B, 0xF0, 0x0C, 0x20, 0x14, 0x40, 0xB8, 0xB0, 0xB4, 0xA8, 0x8C, 0x9C, 0xF0, 0x04,
    0x28, 0x51, 0x79, 0x1D, 0x30, 0x14, 0x38, 0xB2, 0x82, 0xAB, 0xD0, 0x98, 0x2C, 0x50, 0x50, 0x78,
    0x78, 0x9B, 0xF1, 0x1A, 0xB0, 0xF0, 0x8A, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x8B, 0x29, 0x51, 0x79,
    0x8A, 0x24, 0x70, 0x59, 0x8B, 0x20, 0x58, 0x71, 0x8A, 0x44, 0x69, 0x38, 0x8B, 0x39, 0x40, 0x68,
    0x8A, 0x64, 0x48, 0x31, 0x8B, 0x30, 0x49, 0x60, 0xA5, 0x88, 0x20, 0x09, 0x71, 0x58, 0x44, 0x68,

    // bank 6, 256 bytes
    0x11, 0x39, 0x64, 0x49, 0x30, 0x19, 0xF1, 0xAC, 0x00, 0x2C, 0x54, 0x7C, 0xF0, 0x8C, 0xA8, 0x04,
    0x28, 0x50, 0x78, 0xF1, 0x88, 0x97, 0x26, 0xA8, 0x59, 0x98, 0xAC, 0x8C, 0x02, 0x26, 0x46, 0x66,
    0xF0, 0x89, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x24, 0x70, 0x59, 0x44, 0x69, 0x38, 0x64, 0x48, 0x31,
    0xA9, 0x88, 0x09, 0x20, 0x59, 0x70, 0xAB, 0x11, 0x38, 0x40, 0x69, 0xA8, 0x19, 0x31, 0x48, 0x60,
    0x8C, 0xA8, 0x3C, 0x41, 0x5C, 0x20, 0x7C, 0x00, 0xF1, 0x87, 0x98, 0x19, 0x86, 0xA8, 0x6E, 0x76,
    0x7E, 0xA9, 0x99, 0x88, 0x2D, 0x55, 0x7D, 0x9E, 0xB9, 0xA3, 0x8A, 0x22, 0x8A, 0x6E, 0x8A, 0x56,
    0x8A, 0x5E, 0x9F, 0xB1, 0x83, 0x06, 0x26, 0x46, 0x66, 0x0E, 0x2E, 0x4E, 0x6E, 0x9D, 0xB8, 0xAD,
    0x00, 0x2C, 0x54, 0x7C, 0xF2, 0xB1, 0x8C, 0xB4, 0x99, 0xB9, 0xA3, 0x2D, 0x55, 0x7D, 0x81, 0x91,
    0xAC, 0x38, 0xAD, 0x3A, 0xB5, 0x83, 0x91, 0xAC, 0x2D, 0xD9, 0x28, 0xD8, 0x4D, 0xD9, 0x48, 0xD8,
    0x6D, 0xD9, 0x68, 0xD8, 0x8C, 0x9D, 0xAE, 0x29, 0xD9, 0x04, 0xAE, 0xD8, 0x51, 0xD9, 0x04, 0xAE,
    0xD8, 0x79, 0xD9, 0x04, 0xD8, 0x81, 0xF3, 0x9D, 0xAD, 0x00, 0x8D, 0xAE, 0x19, 0x81, 0xAD, 0xD9,
    0x01, 0xD8, 0xF2, 0xAE, 0xDA, 0x26, 0xD8, 0x8E, 0x91, 0x29, 0x83, 0xA7, 0xD9, 0xAD, 0xAD, 0xAD,
    0xAD, 0xF3, 0x2A, 0xD8, 0xD8, 0xF1, 0xB0, 0xAC, 0x89, 0x91, 0x3E, 0x5E, 0x76, 0xF3, 0xAC, 0x2E,
    0x2E, 0xF1, 0xB1, 0x8C, 0x5A, 0x9C, 0xAC, 0x2C, 0x28, 0x28, 0x28, 0x9C, 0xAC, 0x30, 0x18, 0xA8,
    0x98, 0x81, 0x28, 0x34, 0x3C, 0x97, 0x24, 0xA7, 0x28, 0x34, 0x3C, 0x9C, 0x24, 0xF2, 0xB0, 0x89,
    0xAC, 0x91, 0x2C, 0x4C, 0x6C, 0x8A, 0x9B, 0x2D, 0xD9, 0xD8, 0xD8, 0x51, 0xD9, 0xD8, 0xD8, 0x79,

    // bank 7, 138 bytes (remainder)
    0xD9, 0xD8, 0xD8, 0xF1, 0x9E, 0x88, 0xA3, 0x31, 0xDA, 0xD8, 0xD8, 0x91, 0x2D, 0xD9, 0x28, 0xD8,
    0x4D, 0xD9, 0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1, 0x83, 0x93, 0x35, 0x3D, 0x80, 0x25, 0xDA,
    0xD8, 0xD8, 0x85, 0x69, 0xDA, 0xD8, 0xD8, 0xB4, 0x93, 0x81, 0xA3, 0x28, 0x34, 0x3C, 0xF3, 0xAB,
    0x8B, 0xF8, 0xA3, 0x91, 0xB6, 0x09, 0xB4, 0xD9, 0xAB, 0xDE, 0xFA, 0xB0, 0x87, 0x9C, 0xB9, 0xA3,
    0xDD, 0xF1, 0xA3, 0xA3, 0xA3, 0xA3, 0x95, 0xF1, 0xA3, 0xA3, 0xA3, 0x9D, 0xF1, 0xA3, 0xA3, 0xA3,
    0xA3, 0xF2, 0xA3, 0xB4, 0x90, 0x80, 0xF2, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3,
    0xA3, 0xB2, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xB0, 0x87, 0xB5, 0x99, 0xF1, 0xA3, 0xA3, 0xA3,
    0x98, 0xF1, 0xA3, 0xA3, 0xA3, 0xA3, 0x97, 0xA3, 0xA3, 0xA3, 0xA3, 0xF3, 0x9B, 0xA3, 0xA3, 0xDC,
    0xB9, 0xA7, 0xF1, 0x26, 0x26, 0x26, 0xD8, 0xD8, 0xFF
};

// thanks to Noah Zerkin for piecing this stuff together!
const unsigned char dmpConfig[MPU6050_DMP_CONFIG_SIZE] = {
//  BANK    OFFSET  LENGTH  [DATA]
    0x03,   0x7B,   0x03,   0x4C, 0xCD, 0x6C,         // FCFG_1 inv_set_gyro_calibration
    0x03,   0xAB,   0x03,   0x36, 0x56, 0x76,         // FCFG_3 inv_set_gyro_calibration
    0x00,   0x68,   0x04,   0x02, 0xCB, 0x47, 0xA2,   // D_0_104 inv_set_gyro_calibration
    0x02,   0x18,   0x04,   0x00, 0x05, 0x8B, 0xC1,   // D_0_24 inv_set_gyro_calibration
    0x01,   0x0C,   0x04,   0x00, 0x00, 0x00, 0x00,   // D_1_152 inv_set_accel_calibration
    0x03,   0x7F,   0x06,   0x0C, 0xC9, 0x2C, 0x97, 0x97, 0x97, // FCFG_2 inv_set_accel_calibration
    0x03,   0x89,   0x03,   0x26, 0x46, 0x66,         // FCFG_7 inv_set_accel_calibration
    0x00,   0x6C,   0x02,   0x20, 0x00,               // D_0_108 inv_set_accel_calibration
    0x02,   0x40,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_00 inv_set_compass_calibration
    0x02,   0x44,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_01
    0x02,   0x48,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_02
    0x02,   0x4C,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_10
    0x02,   0x50,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_11
    0x02,   0x54,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_12
    0x02,   0x58,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_20
    0x02,   0x5C,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_21
    0x02,   0xBC,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_22
    0x01,   0xEC,   0x04,   0x00, 0x00, 0x40, 0x00,   // D_1_236 inv_apply_endian_accel
    0x03,   0x7F,   0x06,   0x0C, 0xC9, 0x2C, 0x97, 0x97, 0x97, // FCFG_2 inv_set_mpu_sensors
    0x04,   0x02,   0x03,   0x0D, 0x35, 0x5D,         // CFG_MOTION_BIAS inv_turn_on_bias_from_no_motion
    0x04,   0x09,   0x04,   0x87, 0x2D, 0x35, 0x3D,   // FCFG_5 inv_set_bias_update
    0x00,   0xA3,   0x01,   0x00,                     // D_0_163 inv_set_dead_zone
                 // SPECIAL 0x01 = enable interrupts
    0x00,   0x00,   0x00,   0x01, // SET INT_ENABLE at i=22, SPECIAL INSTRUCTION
    0x07,   0x86,   0x01,   0xFE,                     // CFG_6 inv_set_fifo_interupt
    0x07,   0x41,   0x05,   0xF1, 0x20, 0x28, 0x30, 0x38, // CFG_8 inv_send_quaternion
    0x07,   0x7E,   0x01,   0x30,                     // CFG_16 inv_set_footer
    0x07,   0x46,   0x01,   0x9A,                     // CFG_GYRO_SOURCE inv_send_gyro
    0x07,   0x47,   0x04,   0xF1, 0x28, 0x30, 0x38,   // CFG_9 inv_send_gyro -> inv_construct3_fifo
    0x07,   0x6C,   0x04,   0xF1, 0x28, 0x30, 0x38,   // CFG_12 inv_send_accel -> inv_construct3_fifo
    0x02,   0x16,   0x02,   0x00, 0x01                // D_0_22 inv_set_fifo_rate

    // This very last 0x01 WAS a 0x09, which drops the FIFO rate down to 20 Hz. 0x07 is 25 Hz,
    // 0x01 is 100Hz. Going faster than 100Hz (0x00=200Hz) tends to result in very noisy data.
    // DMP output frequency is calculated easily using this equation: (200Hz / (1 + value))

    // It is important to make sure the host processor can keep up with reading and processing
    // the FIFO output at the desired rate. Handling FIFO overflow cleanly is also a good idea.
};

const unsigned char dmpUpdates[MPU6050_DMP_UPDATES_SIZE] = {
    0x01,   0xB2,   0x02,   0xFF, 0xFF,
    0x01,   0x90,   0x04,   0x09, 0x23, 0xA1, 0x35,
    0x01,   0x6A,   0x02,   0x06, 0x00,
    0x01,   0x60,   0x08,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00,   0x60,   0x04,   0x40, 0x00, 0x00, 0x00,
    0x01,   0x62,   0x02,   0x00, 0x00,
    0x00,   0x60,   0x04,   0x00, 0x40, 0x00, 0x00
};


static unsigned char I2C_readBits(unsigned char regAddr, unsigned char bitStart, unsigned char length) {

	// 01101001 read byte
	// 76543210 bit numbers
	//    xxx   args: bitStart=4, length=3
	//    010   masked
	//   -> 010 shifted
	unsigned char mask,tmp;
	tmp = I2C_readByte(regAddr);
	mask = ((1 << length) - 1) << (bitStart - length + 1);
	tmp &= mask;
	tmp >>= (bitStart - length + 1);
	return tmp;
}

static void I2C_writeBits(unsigned char regAddr, unsigned char bitStart, unsigned char length, unsigned char data) {

	//      010 value to write
	// 76543210 bit numbers
	//    xxx   args: bitStart=4, length=3
	// 00011100 mask byte
	// 10101111 original value (sample)
	// 10100011 original & ~mask
	// 10101011 masked | value
	unsigned char tmp,mask;
	tmp = I2C_readByte(regAddr);
	mask = ((1 << length) - 1) << (bitStart - length + 1);
	data <<= (bitStart - length + 1);
	data &= mask;
	tmp &= ~(mask);
	tmp |= data;
	I2C_writeByte(regAddr, tmp);
}

static void I2C_writeBit(unsigned char regAddr, unsigned char bitNum, unsigned char data) {
	unsigned char tmp;
	tmp = I2C_readByte(regAddr);
	tmp = (data != 0) ? (tmp | (1 << bitNum)) : (tmp & ~(1 << bitNum));
	I2C_writeByte(regAddr,tmp);
}

// static void I2C_writeBytes(unsigned char writeAddr, unsigned char length, unsigned char *data) {
// 	while(length--){
// 		I2C_writeByte(writeAddr++, *data++);
// 	}
// }

static void I2C_readBytes(unsigned char readAddr, unsigned char length, unsigned char *data){
	while(length--) {
		*data = I2C_readByte(readAddr++);
	}
}

static void MPUsetMemoryBank(unsigned char bank, bool prefetchEnabled, bool userBank) {
    bank &= 0x1F;	// Clean bit 7~5, only keep bit 0~4
    if (userBank)
		bank |= 0x20;	// Enable bit 5
    if (prefetchEnabled)
		bank |= 0x40;	// Enable bit 6
    I2C_writeByte(MPU6050_RA_BANK_SEL, bank);
}

static void MPUsetMemoryStartAddress(unsigned char address) {
    I2C_writeByte(MPU6050_RA_MEM_START_ADDR, address);
}

static unsigned char MPUreadMemoryByte() {
    return I2C_readByte(MPU6050_RA_MEM_R_W);
}

static unsigned char MPUgetXGyroOffset() {
    return I2C_readBits(MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH);
}

static unsigned char MPUgetYGyroOffset() {
    return I2C_readBits(MPU6050_RA_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH);
}

static unsigned char MPUgetZGyroOffset() {
    return I2C_readBits(MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH);
}

static void MPUsetSlaveAddress(unsigned char num, unsigned char address) {
    if (num > 3) return;
    I2C_writeByte(MPU6050_RA_I2C_SLV0_ADDR + num*3, address);
}

static void MPUsetI2CMasterModeEnabled(bool enabled) {
    I2C_writeBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

static void MPUresetI2CMaster() {
    I2C_writeBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_RESET_BIT, TRUE);
}

static bool MPUwriteMemoryBlock(const unsigned char *data, unsigned short dataSize, unsigned char bank, unsigned char address, bool verify, bool useProgMem) {
	/*
	verifyBuffer and progBuffer malloc/free has been removed, ProgMem support removed
	*/
	unsigned char chunkSize;
	unsigned short i;
	MPUsetMemoryBank(bank, FALSE, FALSE);
	MPUsetMemoryStartAddress(address);
	for (i = 0; i < dataSize;) {
		// determine correct chunk size according to bank position and data size
		chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

		// make sure we don't go past the data size
		if (i + chunkSize > dataSize)
			chunkSize = dataSize - i;

		// make sure this chunk doesn't go past the bank boundary (256 bytes)
		if (chunkSize > 256 - address)
			chunkSize = 256 - address;

		// write the chunk of data as specified
		//        MPUprogBuffer = (unsigned char *)data + i;

		I2C_writeBytes(MPU6050_RA_MEM_R_W, chunkSize, (unsigned char *)data + i);

		// verify data if needed
		if (verify) {
			// MPUsetMemoryBank(bank, FALSE, FALSE);
			// MPUsetMemoryStartAddress(address);
			// I2C_readBytes(MPU6050_RA_MEM_R_W, chunkSize, MPUverifyBuffer);
			// if (memcmp((unsigned char *)data + i, MPUverifyBuffer, chunkSize) != 0) {
			// 	return FALSE; // uh oh.
			// }
		}

		// increase byte index by [chunkSize]
		i += chunkSize;

		// unsigned char automatically wraps to 0 at 256
		address += chunkSize;

		// if we aren't done, update bank (if necessary) and address
		if (i < dataSize) {
			if (address == 0)
				bank++;
			MPUsetMemoryBank(bank, FALSE, FALSE);
			MPUsetMemoryStartAddress(address);
		}
	}
	return TRUE;
}

static bool MPUwriteProgMemoryBlock(const unsigned char *data, unsigned short dataSize, unsigned char bank, unsigned char address, bool verify) {
    return MPUwriteMemoryBlock(data, dataSize, bank, address, verify, TRUE);
}

static bool MPUwriteDMPConfigurationSet(const unsigned char *data, unsigned short dataSize, bool useProgMem) {
    unsigned char success, special;
    unsigned short i;

    // config set data is a long string of blocks with the following structure:
    // [bank] [offset] [length] [byte[0], byte[1], ..., byte[length]]
    unsigned char bank, offset, length;
    for (i = 0; i < dataSize;) {
				bank = data[i++];
				offset = data[i++];
				length = data[i++];

        // write data or perform special action
        if (length > 0) {
 //           MPUprogBuffer = (uint8_t *)data + i;
					/* too few arguments in function call? added FALSE at the end */
            success = MPUwriteMemoryBlock((unsigned char *)data + i, length, bank, offset, TRUE, FALSE);
            i += length;
        } else {
            // special instruction
            // NOTE: this kind of behavior (what and when to do certain things)
            // is totally undocumented. This code is in here based on observed
            // behavior only, and exactly why (or even whether) it has to be here
            // is anybody's guess for now.
            special = data[i++];
            /*Serial.print("Special command code ");
            Serial.print(special, HEX);
            Serial.println(" found...");*/
            if (special == 0x01) {
                // enable DMP-related interrupts

                //setIntZeroMotionEnabled(TRUE);
                //setIntFIFOBufferOverflowEnabled(TRUE);
                //setIntDMPEnabled(TRUE);
                I2C_writeByte(MPU6050_RA_INT_ENABLE, 0x32);  // single operation

                success = TRUE;
            } else {
                // unknown special command
                success = FALSE;
            }
        }

        if (!success) {
            return FALSE; // uh oh
        }
    }
    return TRUE;
}


static bool MPUwriteProgDMPConfigurationSet(const unsigned char *data, unsigned short dataSize) {
    return MPUwriteDMPConfigurationSet(data, dataSize, TRUE);
}

/** Set clock source setting.
 * An internal 8MHz oscillator, gyroscope based clock, or external sources can
 * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
 * or an external source is chosen as the clock source, the MPU-60X0 can operate
 * in low power modes with the gyroscopes disabled.
 *
 * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
 * However, it is highly recommended that the device be configured to use one of
 * the gyroscopes (or an external clock source) as the clock reference for
 * improved stability. The clock source can be selected according to the following table:
 *
 * <pre>
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 * </pre>
 *
 * @param source New clock source setting
 * @see getClockSource()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_CLKSEL_BIT
 * @see MPU6050_PWR1_CLKSEL_LENGTH
 */
static void MPUsetClockSource(unsigned char source) {
    I2C_writeBits(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

/** Set full interrupt enabled status.
 * Full register byte for all interrupts, for quick reading. Each bit should be
 * set 0 for disabled, 1 for enabled.
 * @param enabled New interrupt enabled status
 * @see getIntFreefallEnabled()
 * @see MPU6050_RA_INT_ENABLE
 * @see MPU6050_INTERRUPT_FF_BIT
 **/
static void MPUsetIntEnabled(unsigned char enabled) {
    I2C_writeByte(MPU6050_RA_INT_ENABLE, enabled);
}

/** Set gyroscope sample rate divider.
 * @param rate New sample rate divider
 * @see getRate()
 * @see MPU6050_RA_SMPLRT_DIV
 */
static void MPUsetRate(unsigned char rate) {
    I2C_writeByte(MPU6050_RA_SMPLRT_DIV, rate);
}

/** Set external FSYNC configuration.
 * @see getExternalFrameSync()
 * @see MPU6050_RA_CONFIG
 * @param sync New FSYNC configuration value
 */
static void MPUsetExternalFrameSync(unsigned char sync) {
    I2C_writeBits(MPU6050_RA_CONFIG, MPU6050_CFG_EXT_SYNC_SET_BIT, MPU6050_CFG_EXT_SYNC_SET_LENGTH, sync);
}

/** Set digital low-pass filter configuration.
 * @param mode New DLFP configuration setting
 * @see getDLPFBandwidth()
 * @see MPU6050_DLPF_BW_256
 * @see MPU6050_RA_CONFIG
 * @see MPU6050_CFG_DLPF_CFG_BIT
 * @see MPU6050_CFG_DLPF_CFG_LENGTH
 */
static void MPUsetDLPFMode(unsigned char mode) {
    I2C_writeBits(MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
static void MPUsetFullScaleGyroRange(unsigned char range) {
    I2C_writeBits(MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

// DMP_CFG_1 register

// static unsigned char MPUgetDMPConfig1() {
//     return I2C_readByte(MPU6050_RA_DMP_CFG_1);
// }
static void MPUsetDMPConfig1(unsigned char config) {
    I2C_writeByte(MPU6050_RA_DMP_CFG_1, config);
}

// DMP_CFG_2 register

// static unsigned char MPUgetDMPConfig2() {
//     return I2C_readByte(MPU6050_RA_DMP_CFG_2);
// }
static void MPUsetDMPConfig2(unsigned char config) {
    I2C_writeByte(MPU6050_RA_DMP_CFG_2, config);
}

static void MPUsetOTPBankValid(bool enabled) {
    I2C_writeBit(MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT, enabled);
}

static void MPUsetXGyroOffset(unsigned char offset) {
    I2C_writeBits(MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

static void MPUsetYGyroOffset(unsigned char offset) {
    I2C_writeBits(MPU6050_RA_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

static void MPUsetZGyroOffset(unsigned char offset) {
    I2C_writeBits(MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

static void MPUsetXGyroOffsetUser(unsigned short offset) {
    I2C_writeWord(MPU6050_RA_XG_OFFS_USRH, offset);
}

static void MPUsetYGyroOffsetUser(unsigned short offset) {
    I2C_writeWord(MPU6050_RA_YG_OFFS_USRH, offset);
}

static void MPUsetZGyroOffsetUser(unsigned short offset) {
    I2C_writeWord(MPU6050_RA_ZG_OFFS_USRH, offset);
}

/** Reset the FIFO.
 * This bit resets the FIFO buffer when set to 1 while FIFO_EN equals 0. This
 * bit automatically clears to 0 after the reset has been triggered.
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_FIFO_RESET_BIT
 */
void MPUresetFIFO() {
    I2C_writeBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, TRUE);
}

// FIFO_COUNT* registers

/** Get current FIFO buffer size.
 * This value indicates the number of bytes stored in the FIFO buffer. This
 * number is in turn the number of bytes that can be read from the FIFO buffer
 * and it is directly proportional to the number of samples available given the
 * set of sensor data bound to be stored in the FIFO (register 35 and 36).
 * @return Current FIFO buffer size
 */
unsigned short MPUgetFIFOCount() {
    I2C_readBytes(MPU6050_RA_FIFO_COUNTH, 2, MPUbuffer);
    // return (((unsigned short)MPUbuffer[0]) << 8) | MPUbuffer[1];//sercan
	unsigned short *arrayPointer = (unsigned short *)&MPUbuffer[0];
	return *arrayPointer;
	  //return (MPUbuffer[0]);

}

/** Set free-fall event acceleration threshold.
 * @param threshold New motion detection acceleration threshold value (LSB = 2mg)
 * @see getMotionDetectionThreshold()
 * @see MPU6050_RA_MOT_THR
 */
static void MPUsetMotionDetectionThreshold(unsigned char threshold) {
    I2C_writeByte(MPU6050_RA_MOT_THR, threshold);
}

/** Set zero motion detection event acceleration threshold.
 * @param threshold New zero motion detection acceleration threshold value (LSB = 2mg)
 * @see getZeroMotionDetectionThreshold()
 * @see MPU6050_RA_ZRMOT_THR
 */
static void MPUsetZeroMotionDetectionThreshold(unsigned char threshold) {
    I2C_writeByte(MPU6050_RA_ZRMOT_THR, threshold);
}

/** Set motion detection event duration threshold.
 * @param duration New motion detection duration threshold value (LSB = 1ms)
 * @see getMotionDetectionDuration()
 * @see MPU6050_RA_MOT_DUR
 */
static void MPUsetMotionDetectionDuration(unsigned char duration) {
    I2C_writeByte(MPU6050_RA_MOT_DUR, duration);
}

/** Set zero motion detection event duration threshold.
 * @param duration New zero motion detection duration threshold value (LSB = 1ms)
 * @see getZeroMotionDetectionDuration()
 * @see MPU6050_RA_ZRMOT_DUR
 */
static void MPUsetZeroMotionDetectionDuration(unsigned char duration) {
    I2C_writeByte(MPU6050_RA_ZRMOT_DUR, duration);
}

/** Set FIFO enabled status.
 * @param enabled New FIFO enabled status
 * @see getFIFOEnabled()
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_FIFO_EN_BIT
 */
static void MPUsetFIFOEnabled(bool enabled) {
    I2C_writeBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT, enabled);
}

void MPUsetDMPEnabled(bool enabled) {
    I2C_writeBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT, enabled);
}

static void MPUresetDMP() {
    I2C_writeBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_RESET_BIT, TRUE);
}

// FIFO_R_W register

/** Get byte from FIFO buffer.
 * This register is used to read and write data from the FIFO buffer. Data is
 * written to the FIFO in order of register number (from lowest to highest). If
 * all the FIFO enable flags (see below) are enabled and all External Sensor
 * Data registers (Registers 73 to 96) are associated with a Slave device, the
 * contents of registers 59 through 96 will be written in order at the Sample
 * Rate.
 *
 * The contents of the sensor data registers (Registers 59 to 96) are written
 * into the FIFO buffer when their corresponding FIFO enable flags are set to 1
 * in FIFO_EN (Register 35). An additional flag for the sensor data registers
 * associated with I2C Slave 3 can be found in I2C_MST_CTRL (Register 36).
 *
 * If the FIFO buffer has overflowed, the status bit FIFO_OFLOW_INT is
 * automatically set to 1. This bit is located in INT_STATUS (Register 58).
 * When the FIFO buffer has overflowed, the oldest data will be lost and new
 * data will be written to the FIFO.
 *
 * If the FIFO buffer is empty, reading this register will return the last byte
 * that was previously read from the FIFO until new data is available. The user
 * should check FIFO_COUNT to ensure that the FIFO buffer is not read when
 * empty.
 *
 * @return Byte from FIFO buffer
 */
// static unsigned char MPUgetFIFOByte() {
//     I2C_readByte(MPU6050_RA_FIFO_R_W, MPUbuffer);
//     return MPUbuffer[0];
// }
void MPUgetFIFOBytes(unsigned char *data, unsigned char length) {
    I2C_readBytes(MPU6050_RA_FIFO_R_W, length, data);
}

static void MPUreadMemoryBlock(unsigned char *data, unsigned short dataSize, unsigned char bank, unsigned char address) {
	unsigned char chunkSize;
	unsigned short i;
	MPUsetMemoryBank(bank, FALSE, FALSE);
    MPUsetMemoryStartAddress(address);

    for (i = 0; i < dataSize;) {
        // determine correct chunk size according to bank position and data size
        chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunkSize > dataSize) chunkSize = dataSize - i;

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunkSize > 256 - address) chunkSize = 256 - address;

        // read the chunk of data as specified
        I2C_readBytes(MPU6050_RA_MEM_R_W, chunkSize, data + i);

        // increase byte index by [chunkSize]
        i += chunkSize;

        // uint8_t automatically wraps to 0 at 256
        address += chunkSize;

        // if we aren't done, update bank (if necessary) and address
        if (i < dataSize) {
            if (address == 0) bank++;
            MPUsetMemoryBank(bank, FALSE, FALSE);
            MPUsetMemoryStartAddress(address);
        }
    }
}

// INT_STATUS register

/** Get full set of interrupt status bits.
 * These bits clear to 0 after the register has been read. Very useful
 * for getting multiple INT statuses, since each single bit read clears
 * all of them because it has to read the whole byte.
 * @return Current interrupt status
 * @see MPU6050_RA_INT_STATUS
 */
unsigned char MPUgetIntStatus() {
    return I2C_readByte(MPU6050_RA_INT_STATUS);
}


unsigned char DMP_Initialize() {

	print("\r\nResetting MPU6050...");
	I2C_writeByte(PWR_MGMT_1, I2C_readByte(PWR_MGMT_1)|1<<7);// Set DEVICE RESET BIT True
	delay_ms(30);	// wait after reset

	print("\r\nDisabling sleep mode...");
	I2C_writeByte(PWR_MGMT_1, I2C_readByte(PWR_MGMT_1)&~(1<<6));// Disable sleep mode

	// get MPU hardware revision
	print("\r\nSelecting user bank 16...");
	MPUsetMemoryBank(0x10, TRUE, TRUE);
	print("\r\nSelecting memory byte 6...");
	MPUsetMemoryStartAddress(0x06);
	print("\r\nChecking hardware revision...");

	print("\r\nRevision @ user[16][6] = ");
	printChar(MPUreadMemoryByte());
	print("\r\nResetting memory bank selection to 0...");
	MPUsetMemoryBank(0, FALSE, FALSE);

	// get X/Y/Z gyro offsets
	unsigned char xgOffset, ygOffset, zgOffset;
    print("\r\nReading gyro offset values...");
    xgOffset = MPUgetXGyroOffset();
    ygOffset = MPUgetYGyroOffset();
    zgOffset = MPUgetZGyroOffset();
    print("\r\nX gyro offset = ");
	printChar(xgOffset);
    print("\r\nY gyro offset = ");
	printChar(ygOffset);
    print("\r\nZ gyro offset = ");
	printChar(zgOffset);

	// setup weird slave stuff (?)
    print("\r\nSetting slave 0 address to 0x7F...");
    MPUsetSlaveAddress(0, 0x7F);
    print("\r\nDisabling I2C Master mode...");
    MPUsetI2CMasterModeEnabled(FALSE);
    print("\r\nSetting slave 0 address to 0x68 (self)...");
    MPUsetSlaveAddress(0, 0x68);
    print("\r\nResetting I2C Master control...");
    MPUresetI2CMaster();
    delay_ms(20);

	// load DMP code into memory banks
	print("\r\nWriting DMP code to MPU memory banks (");
	printChar(MPU6050_DMP_CODE_SIZE);
	print(" bytes)");
	if (MPUwriteProgMemoryBlock(dmpMemory, MPU6050_DMP_CODE_SIZE, 0, 0, TRUE)) {
		print("\r\nSuccess! DMP code written and verified.");

		// write DMP configuration
		print("\nWriting DMP configuration to MPU memory banks (");
		printChar(MPU6050_DMP_CONFIG_SIZE);
		print(" bytes in config def)");
		if (MPUwriteProgDMPConfigurationSet(dmpConfig, MPU6050_DMP_CONFIG_SIZE)) {
			print("\r\nSuccess! DMP configuration written and verified.");

			print("\r\nSetting clock source to Z Gyro...");
			MPUsetClockSource(MPU6050_CLOCK_PLL_ZGYRO);

			print("\r\nSetting DMP and FIFO_OFLOW interrupts enabled...");
			MPUsetIntEnabled(0x12);

			print("\r\nSetting sample rate to 200Hz...");
            MPUsetRate(4); // 1khz / (1 + 4) = 200 Hz

			print("\r\nSetting external frame sync to TEMP_OUT_L[0]...");
            MPUsetExternalFrameSync(MPU6050_EXT_SYNC_TEMP_OUT_L);

			print("\r\nSetting DLPF bandwidth to 42Hz...");
            MPUsetDLPFMode(MPU6050_DLPF_BW_42);

			print("\r\nSetting gyro sensitivity to +/- 2000 deg/sec...");
            MPUsetFullScaleGyroRange(MPU6050_GYRO_FS_2000);

			print("\r\nSetting DMP configuration bytes (function unknown)...");
            MPUsetDMPConfig1(0x03);
            MPUsetDMPConfig2(0x00);

			print("\r\nClearing OTP Bank flag...");
            MPUsetOTPBankValid(FALSE);

			print("\r\nSetting X/Y/Z gyro offsets to previous values...");
            MPUsetXGyroOffset(xgOffset);
            MPUsetYGyroOffset(ygOffset);
            MPUsetZGyroOffset(zgOffset);

			print("\r\nSetting X/Y/Z gyro user offsets to zero...");
            MPUsetXGyroOffsetUser(0);
            MPUsetYGyroOffsetUser(0);
            MPUsetZGyroOffsetUser(0);

			print("\r\nWriting final memory update 1/7 (function unknown)...");

			unsigned char dmpUpdate[16], j;
			unsigned short pos = 0;
			for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            MPUwriteMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], TRUE, FALSE);

            print("\r\nWriting final memory update 2/7 (function unknown)...");
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            MPUwriteMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], TRUE, FALSE);

            print("\r\nResetting FIFO...");
			MPUresetFIFO();

			print("\r\nReading FIFO count...");
			unsigned short fifoCount;
            fifoCount = MPUgetFIFOCount();//sercan
			print("\r\nCurrent FIFO count=");
			printChar(fifoCount);

			print("\r\nSetting motion detection threshold to 2...");
            MPUsetMotionDetectionThreshold(2);

			print("\r\nSetting zero-motion detection threshold to 156...");
            MPUsetZeroMotionDetectionThreshold(156);

			print("\r\nSetting motion detection duration to 80...");
            MPUsetMotionDetectionDuration(80);

			print("\r\nSetting zero-motion detection duration to 0...");
            MPUsetZeroMotionDetectionDuration(0);

			print("\r\nResetting FIFO...");
            MPUresetFIFO();

			print("\r\nEnabling FIFO...");
            MPUsetFIFOEnabled(TRUE);

			print("\r\nEnabling DMP...");
            MPUsetDMPEnabled(TRUE);

			print("\r\nResetting DMP...");
            MPUresetDMP();

			print("\r\nWriting final memory update 3/7 (function unknown)...");
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            MPUwriteMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], TRUE, FALSE);

			print("\r\nWriting final memory update 4/7 (function unknown)...");
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            MPUwriteMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], TRUE, FALSE);

			print("\r\nWriting final memory update 5/7 (function unknown)...");
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
            MPUwriteMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], TRUE, FALSE);

            print("\r\nWaiting for FIFO count > 2...");
            while ((fifoCount = MPUgetFIFOCount()) < 3);

			print("\r\nCurrent FIFO count=");
			printChar(fifoCount);
            print("\r\nReading FIFO data...");
			unsigned char fifoBuffer[128];
            MPUgetFIFOBytes(fifoBuffer, fifoCount);
// #ifdef MPUDEBUG
//             printf("\nReading interrupt status...");
//             mpuIntStatus = MPUgetIntStatus();
//             printf("\nCurrent interrupt status=%x", mpuIntStatus);
// #endif

			print("\r\nReading final memory update 6/7 (function unknown)...");
			for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
			MPUreadMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

			print("\r\nWaiting for FIFO count > 2...");
            while ((fifoCount = MPUgetFIFOCount()) < 3);

			print("\r\nCurrent FIFO count=");
			printChar(fifoCount);

            print("\r\nReading FIFO data...");
            MPUgetFIFOBytes(fifoBuffer, fifoCount);

// #ifdef MPUDEBUG
//             printf("\nReading interrupt status...");
//             mpuIntStatus = MPUgetIntStatus();
//
//             printf("\nCurrent interrupt status=%x", mpuIntStatus);
// #endif

			print("\r\nWriting final memory update 7/7 (function unknown)...");
			for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = dmpUpdates[pos];
			MPUwriteMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1], TRUE, FALSE);

			print("\r\nDMP is good to go! Finally.");

			print("\r\nDisabling DMP (you turn it on later)...");
			MPUsetDMPEnabled(FALSE);

			print("\r\nSetting up internal 42-byte (default) DMP packet buffer...");

				MPUgetFIFOBytes(fifoBuffer, fifoCount);
				print("\r\nCurrent FIFO count=");
				printChar(fifoCount);

			print("\r\nResetting FIFO and clearing INT status one last time...");
            MPUresetFIFO();

				MPUgetFIFOBytes(fifoBuffer, fifoCount);
				print("\r\nCurrent FIFO count=");
				printChar(fifoCount);

			MPUgetIntStatus();

		} else {
			print("\r\nERROR! DMP configuration verification failed.");
			return 2; // configuration block loading failed
		}
	} else {
        print("\r\nERROR! DMP code verification failed.");
        return 1; // main binary block loading failed
    }

	return 0; // success

}

/** Set full-scale accelerometer range.
 * @param range New full-scale accelerometer range setting
 * @see getFullScaleAccelRange()
 */
static void MPUsetFullScaleAccelRange(unsigned char range) {
    I2C_writeBits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

/** Set sleep mode status.
 * @param enabled New sleep mode enabled status
 * @see getSleepEnabled()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
static void MPUsetSleepEnabled(bool enabled) {
    I2C_writeBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/** Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up). This function also sets both the accelerometer and the gyroscope
 * to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets
 * the clock source to use the X Gyro for reference, which is slightly better than
 * the default internal clock source.
 */
void MPUinitialize() {
    MPUsetClockSource(MPU6050_CLOCK_PLL_XGYRO);
    MPUsetFullScaleGyroRange(MPU6050_GYRO_FS_250);
    MPUsetFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    MPUsetSleepEnabled(FALSE); // thanks to Jack Elston for pointing this one out!
}


static unsigned char MPUdmpGetQuaternion16(unsigned short *data, const unsigned char *packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = MPUdmpPacketBuffer;
    data[0] = ((packet[0] << 8) + packet[1]);
    data[1] = ((packet[4] << 8) + packet[5]);
    data[2] = ((packet[8] << 8) + packet[9]);
    data[3] = ((packet[12] << 8) + packet[13]);
    return 0;
}

unsigned char MPUdmpGetQuaternion(Quaternion *q, const unsigned char *packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    unsigned short qI[4];
    unsigned char status = MPUdmpGetQuaternion16(qI, packet);
    if (status == 0) {
        q -> w = (float)qI[0] / 16384.0f;
        q -> x = (float)qI[1] / 16384.0f;
        q -> y = (float)qI[2] / 16384.0f;
        q -> z = (float)qI[3] / 16384.0f;
        return 0;
    }
    return status; // int16 return value, indicates error if this line is reached
}

unsigned char MPUdmpGetEuler(float *data, Quaternion *q) {
    data[0] = _atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);   // psi (z)
    data[1] = -_asin(2*q -> x*q -> z + 2*q -> w*q -> y);                              // theta (y)
    data[2] = _atan2(2*q -> y*q -> z - 2*q -> w*q -> x, 2*q -> w*q -> w + 2*q -> z*q -> z - 1);   // phi (x)
    return 0;
}
