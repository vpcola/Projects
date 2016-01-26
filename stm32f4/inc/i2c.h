#ifndef __I2C_H__
#define __I2C_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "main.h"

#define I2C1_SDA_PIN_SRC    GPIO_PinSource9
#define I2C1_SCL_PIN_SRC    GPIO_PinSource8
#define I2C1_PORT           GPIOB
#define I2C1_PINS           GPIO_Pin_8 | GPIO_Pin_9
#define I2C1_ADDRESS_MODE   I2C_AcknowledgedAddress_7bit


void I2C1_Config(void);

#endif
