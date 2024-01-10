/*
 * MLX90640_I2C_Driver.h
 *
 *  Created on: Jan 8, 2024
 *      Author: yre
 */

#ifndef INC_MLX90640_I2C_DRIVER_H_
#define INC_MLX90640_I2C_DRIVER_H_

#include "stm32g4xx_hal.h"
#include "MX90640.h"

    void MLX90640_I2CInit();
    int MLX90640_I2CGeneralReset(void);
    int MLX90640_I2CRead(uint8_t slaveAddr,uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data);
    int MLX90640_I2CWrite(uint8_t slaveAddr,uint16_t writeAddress, uint16_t data);
    void MLX90640_I2CFreqSet(int freq);

#endif /* INC_MLX90640_I2C_DRIVER_H_ */
