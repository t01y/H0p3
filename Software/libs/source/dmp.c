#include "dmp.h"

#include "MPU6050.h"
#include "uart.h"

// Portable
#define I2C_readByte(addr)			MPU_Sigle_Read(addr)
#define I2C_writeByte(addr, data)	MPU_Sigle_Write(addr, data)
#define delay_ms					delay_ms
#define print						uart_sendStr
#define printChar(x)				uart_short2char((unsigned short)x)


static unsigned char I2C_readBits(unsigned char regAddr, unsigned char bitStart, unsigned char length){

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

static void I2C_writeBit(unsigned char regAddr, unsigned char bitNum, unsigned char data){
	unsigned char tmp;
	tmp = I2C_readByte(regAddr);
	tmp = (data != 0) ? (tmp | (1 << bitNum)) : (tmp & ~(1 << bitNum));
	I2C_writeByte(regAddr,tmp);
}

static void I2C_writeBytes(unsigned char writeAddr, unsigned char length, unsigned char *data){

	int i=0;
	I2C_start(I2C1, slave_address, I2C_Direction_Transmitter);
	I2C_write(I2C1, writeAddr);
	for(i=0; i<length; i++){
		I2C_write(I2C1, data[i]);
  }
  I2C_stop(I2C1);
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
		MPUsetMemoryBank(bank, FALSE, FALSE);
		MPUsetMemoryStartAddress(address);
		MPUverifyBuffer = I2C_readBytes(MPU6050_RA_MEM_R_W, chunkSize);
		if (memcmp((unsigned char *)data + i, MPUverifyBuffer, chunkSize) != 0) {
			return FALSE; // uh oh.
		}
	}

	// increase byte index by [chunkSize]
	i += chunkSize;

	// unsigned char automatically wraps to 0 at 256
	address += chunkSize;

	// if we aren't done, update bank (if necessary) and address
	if (i < dataSize) {
		if (address == 0) bank++;
			MPUsetMemoryBank(bank, FALSE, FALSE);
			MPUsetMemoryStartAddress(address);
		}
	}
	return TRUE;
}

static bool MPUwriteProgMemoryBlock(const unsigned char *data, unsigned short dataSize, unsigned char bank, unsigned char address, bool verify) {
    return MPUwriteMemoryBlock(data, dataSize, bank, address, verify, TRUE);
}

unsigned char DMP_Initialize() {

	print("\r\nResettubg MPU6050...");
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
	}



	return 1;

}
