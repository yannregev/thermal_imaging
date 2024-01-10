/*
 * MLX90640_I2C_Driver.h
 *
 *  Created on: Jan 8, 2024
 *      Author: yre
 */

#include "MLX90640_I2C_Driver.h"
#include "stm32g4xx_ll_i2c.h"
#include "stm32g4xx_ll_bus.h"
#include "main.h"

//I2C_HandleTypeDef _hi2c;

void MLX90640_I2CInit(/*I2C_HandleTypeDef hi2c*/)
{
//	_hi2c = hi2c;
}

uint16_t MLX90640_I2CReadWord(uint8_t slaveAddress, uint16_t start_address)
{
/*
	uint8_t buf[2];
	HAL_I2C_Mem_Read(&_hi2c, slaveAddress, start_address, 2, buf, 2, 100);
	return ((buf[0] << 8) | (buf[1] & 0xFF));
*/

	volatile uint8_t reg_m,reg_l,dat_m,dat_l;

	reg_m = (uint8_t) ((start_address & 0xFF00) >> 8);	//Address MSB
	reg_l = (uint8_t) (start_address & 0x00FF); 	    //Address LSB


	while (LL_I2C_IsActiveFlag_BUSY(I2C1)) {
	}

    LL_I2C_HandleTransfer(I2C1, slaveAddress, LL_I2C_ADDRSLAVE_7BIT, 2,
    		LL_I2C_MODE_SOFTEND, LL_I2C_GENERATE_START_WRITE);


    while (!LL_I2C_IsActiveFlag_TXIS(I2C1)){};

    LL_I2C_TransmitData8(I2C1, reg_m);

    //LL_I2C_HandleTransfer(I2C1, slaveAddress, LL_I2C_ADDRSLAVE_7BIT, 1, I2C_CR2_AUTOEND, LL_I2C_GENERATE_NOSTARTSTOP);

    while (!LL_I2C_IsActiveFlag_TXE(I2C1)) {};

    LL_I2C_TransmitData8(I2C1, reg_l);

    while (!LL_I2C_IsActiveFlag_TC(I2C1)) {};

    LL_I2C_HandleTransfer(I2C1, slaveAddress, LL_I2C_ADDRSLAVE_7BIT, 2,
                              I2C_CR2_AUTOEND ,LL_I2C_GENERATE_START_READ);

    while (!LL_I2C_IsActiveFlag_RXNE(I2C1)) {};

    dat_m = LL_I2C_ReceiveData8(I2C1);

    while (!LL_I2C_IsActiveFlag_RXNE(I2C1)) {};

    dat_l = LL_I2C_ReceiveData8(I2C1);

    /* No need to Check TC flag, with AUTOEND mode the stop is automatically
     * generated.
     * Wait until STOPF flag is reset */
    while (!LL_I2C_IsActiveFlag_STOP(I2C1)) {};

	/* Clear NACKF Flag */
    LL_I2C_ClearFlag_NACK(I2C1);

    /* Clear STOP Flag */
    LL_I2C_ClearFlag_STOP(I2C1);

    /* Clear Configuration Register 2 */
    I2C1->CR2 &= (uint32_t)~((uint32_t)(I2C_CR2_SADD | I2C_CR2_HEAD10R |
    		I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_RD_WRN));

	return ((uint16_t) (dat_m << 8)) | ((uint16_t)((dat_l) & 0x00FF));

}

int MLX90640_I2CGeneralReset(void)
{
	return 0;
}

int MLX90640_I2CRead(uint8_t slaveAddr,uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data)
{
	slaveAddr <<= 1;

	uint16_t temp_address = startAddress;
	uint16_t temp_data;
	for(int i=0; i < nMemAddressRead; i++){

		temp_data = MLX90640_I2CReadWord(slaveAddr,temp_address);
		temp_address++;
		*(data + i) = temp_data;

	}
	return 0;
}

int MLX90640_I2CWrite(uint8_t slaveAddr,uint16_t writeAddress, uint16_t data)
{
/*
	uint8_t buf[2];
	buf[1] =  data & 0xFF;
	buf[0] = (data >> 8) & 0xFF;
	slaveAddr <<= 1;
	HAL_I2C_Mem_Write(&_hi2c, slaveAddr, writeAddress, 2, buf, 2, 100);
	return 0;
*/
	slaveAddr <<= 1;
	uint8_t reg_m,reg_l,dat_m,dat_l;
	reg_m = (uint8_t) ((writeAddress & 0xFF00) >> 8);			//Address MSB
	reg_l = (uint8_t) (writeAddress & 0x00FF); 				//Address LSB
	dat_m = (uint8_t) ((data & 0xFF00) >> 8);	// Data MSB
	dat_l = (uint8_t) (data & 0x00FF);			//Data LSB

	while (LL_I2C_IsActiveFlag_BUSY(I2C1)) {
	}

    LL_I2C_HandleTransfer(I2C1, slaveAddr, LL_I2C_ADDRSLAVE_7BIT, 2,
    		I2C_CR2_RELOAD, LL_I2C_GENERATE_START_WRITE);


    while (!LL_I2C_IsActiveFlag_TXIS(I2C1)){};

    LL_I2C_TransmitData8(I2C1, reg_m);

    //LL_I2C_HandleTransfer(I2C1, slaveAddress, LL_I2C_ADDRSLAVE_7BIT, 1, I2C_CR2_AUTOEND, LL_I2C_GENERATE_NOSTARTSTOP);

    while (!LL_I2C_IsActiveFlag_TXE(I2C1)) {};

    LL_I2C_TransmitData8(I2C1, reg_l);

    while (!LL_I2C_IsActiveFlag_TCR(I2C1)) {};

    LL_I2C_HandleTransfer(I2C1, slaveAddr, LL_I2C_ADDRSLAVE_7BIT, 2,
                              I2C_CR2_AUTOEND ,LL_I2C_GENERATE_NOSTARTSTOP);

    while (!LL_I2C_IsActiveFlag_TXIS(I2C1)) {};

    LL_I2C_TransmitData8(I2C1, dat_m);

    while (!LL_I2C_IsActiveFlag_TXIS(I2C1)) {};

    LL_I2C_TransmitData8(I2C1, dat_l);

    /* No need to Check TC flag, with AUTOEND mode the stop is automatically
     * generated.
     * Wait until STOPF flag is reset */
    while (!LL_I2C_IsActiveFlag_STOP(I2C1)) {};

	/* Clear NACKF Flag */
    LL_I2C_ClearFlag_NACK(I2C1);

    /* Clear STOP Flag */
    LL_I2C_ClearFlag_STOP(I2C1);

    /* Clear Configuration Register 2 */
    I2C1->CR2 &= (uint32_t)~((uint32_t)(I2C_CR2_SADD | I2C_CR2_HEAD10R |
    		I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_RD_WRN));


	return 0;
}

void MLX90640_I2CFreqSet(int freq)
{
/*
	_i2cPort.Init.ClockSpeed = freq; // Set the desired bus frequency in Hz

    // Initialize the I2C peripheral
    if (HAL_I2C_Init(&hi2c) != HAL_OK) {
        // Initialization Error
        Error_Handler(); // Your error handling mechanism
    }
*/
}
