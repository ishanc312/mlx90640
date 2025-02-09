#include <stdio.h>
#include "MLX90640_I2C_Driver.h"
#include "i2c.h"

void MLX90640_I2CInit() {
	MX_I2C1_Init();
}

void MLX90640_I2CFreqSet(int freq) {
	HAL_I2C_DeInit(&hi2c1);
	hi2c1.Init.Timing = freq;
	HAL_I2C_Init(&hi2c1);

}

int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress,
		uint16_t nMemAddressRead, uint16_t *data) {

	uint8_t* pData = (uint8_t*) data;
	// Need to perform a cast, as STM32 has 8-bit (1 byte) memory addresses
	// And thus is the required type for the MCU memory address parameter in HAL_I2C_Mem_Read()

	int ack = HAL_I2C_Mem_Read(&hi2c1, (slaveAddr << 1), startAddress,
	I2C_MEMADD_SIZE_16BIT, pData, 2 * nMemAddressRead, 500);
	// We read nMemAddressRead words (2 bytes each); thus, the size parameter we pass in is 2*nMemAddressRead

	if (ack != HAL_OK) {
		return -1;
	}

	// Perform Endian Conversion from Big-Endian to Little-Endian
	for (int k = 0; k < nMemAddressRead * 2; k += 2) {
		uint8_t temp = pData[k+1];
		pData[k+1] = pData[k];
		pData[k] = temp;
	}

	return 0;

}

int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data) {
	uint8_t pData[2];
	pData[0] = (uint8_t)((data >> 8) & 0xFF); // MSB
	pData[1] = (uint8_t)(data & 0xFF);
	// Perform Byte Swap; MSB given by (data >> 8) is in pData[0], (data & 0x00FF) is in pData[1]
	// Thus, the 2-byte data is now in Big Endian Order, and it is ready to write to MLX90640

	int ack = HAL_I2C_Mem_Write(&hi2c1, (slaveAddr << 1), writeAddress,
	I2C_MEMADD_SIZE_16BIT, pData, sizeof(pData), 500);

	if (ack != HAL_OK) {
		return -1;
	}

	static uint16_t dataCheck;
	MLX90640_I2CRead(slaveAddr, writeAddress, 2, &dataCheck);
	// We use our previously written I2C_Read function to read what we just wrote and place it in dataCheck
	if (dataCheck != data) {
		// if "data" parameter did not end up getting written correctly, then dataCheck will be unequal to data
		// Then return this error code
		return -2;
	}

	return 0;
	// Otherwise, success!

}

int MLX90640_I2CGeneralReset(uint8_t slaveAddr) {
	uint8_t resetCode = 0x06;
	int ack = HAL_I2C_Mem_Write(&hi2c1, (slaveAddr << 1), 0x00, I2C_MEMADD_SIZE_8BIT,
			&resetCode, sizeof(resetCode), 500);
	if (ack != HAL_OK) {
		return -1;
	}
	// The zero page is 1 byte, i.e. 8 bit address as opposed to the 16 bit memory locations of the rest of MLX90640

	return 0;
}
