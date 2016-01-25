#include "i2c.h"



/**
 * @brief  Configures the I2C1 used for OV9655 camera module configuration.
 * @param  None
 * @retval None
 */
void I2C1_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef  I2C_InitStruct;

    /* I2C1 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    /* GPIOB clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 

    /* Connect I2C1 pins to AF4 ************************************************/
    GPIO_PinAFConfig(I2C1_PORT, I2C1_SDA_PIN_SRC, GPIO_AF_I2C1);
    GPIO_PinAFConfig(I2C1_PORT, I2C1_SCL_PIN_SRC, GPIO_AF_I2C1);  

    /* Configure I2C1 GPIOs *****************************************************/  
    GPIO_InitStructure.GPIO_Pin = I2C1_PINS;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;   
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Configure I2C1 ***********************************************************/  
    /* I2C DeInit */   
    I2C_DeInit(I2C1);

    /* Enable the I2C peripheral */
    I2C_Cmd(I2C1, ENABLE);

    /* Set the I2C structure parameters */
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStruct.I2C_OwnAddress1 = 0xFE;
    I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C1_ADDRESS_MODE;
    I2C_InitStruct.I2C_ClockSpeed = 30000;

    /* Initialize the I2C peripheral w/ selected parameters */
    I2C_Init(I2C1, &I2C_InitStruct);
}



