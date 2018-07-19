
#ifndef __STM32F0xx_MSP_H
#define __STM32F0xx_MSP_H

#include "stm32f0xx_hal.h"

void SPI_TxInit(SPI_HandleTypeDef* hspi);
void SPI_TxDeInit(SPI_HandleTypeDef* hspi);

#endif /* __STM32F0xx_MSP_H */
