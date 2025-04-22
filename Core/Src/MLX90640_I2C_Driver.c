#include "MLX90640_I2C_Driver.h"
#include "stdio.h"
#include "i2c.h"

void MLX90640_I2CInit() {
	MX_I2C1_Init();
}

void MLX90640_I2CFreqSet(int freq) {
	// frequency is in kHz
	HAL_I2C_DeInit(&hi2c1);
	hi2c1.Init.Timing = freq;
	HAL_I2C_Init(&hi2c1);

}

int MLX90640_I2CGeneralReset() {
	uint8_t reset_cmd = 0x06;
	int ack = HAL_I2C_Master_Transmit(&hi2c1, 0x00 << 1, &reset_cmd, 1, HAL_MAX_DELAY);
	if (ack != HAL_OK) {
		return -1;
	}
	return 0;
}

int MLX90640_getDeviceId(uint8_t slaveAddr, uint16_t* device_id) {
	// 4640 should be the Device ID Returned (?) i.e. hopefully some nonzero value
	return MLX90640_I2CRead(slaveAddr, 0x2407, 1, device_id);
}

// Make sure USART2 is enabled (?)
void I2CScan() {
	// Simple helper function to verify that the slaveAddr was still 0x33, factory default
	printf("Scanning I2C Bus\r\n");
	for (uint8_t address = 0; address < 128; address++) {
		if (HAL_I2C_IsDeviceReady(&hi2c1, (address << 1), 1, 100) == HAL_OK) {
			printf("FOUND: 0x%02X\n", address);
		} else {
			printf("NOT FOUND: 0x%02X\n", address);
		}
	}
	printf("Scan Complete\r\n");
}

int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress,
		uint16_t nMemAddressRead, uint16_t *data) {
	uint8_t* pData = (uint8_t*) data;
	int ack = HAL_I2C_Mem_Read(&hi2c1, (slaveAddr << 1), startAddress,
	I2C_MEMADD_SIZE_16BIT, pData, 2 * nMemAddressRead, 500);
	if (ack != HAL_OK) {
		return -1;
	}

	// Perform Endian Conversion on the received data
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
	pData[1] = (uint8_t)(data & 0xFF); // LSB
	// Perform Endian Conversion on data to write

	int ack = HAL_I2C_Mem_Write(&hi2c1, (slaveAddr << 1), writeAddress,
	I2C_MEMADD_SIZE_16BIT, pData, sizeof(pData), 500);

	if (ack != HAL_OK) {
		return -1;
	}

	uint16_t dataCheck;
	MLX90640_I2CRead(slaveAddr, writeAddress, 2, &dataCheck);
	if (dataCheck != data) return -2;

	return 0;
}
