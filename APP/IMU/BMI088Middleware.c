#include "BMI088Middleware.h"
#include "main.h"
#include "cmsis_os.h"
#include "spi.h"
extern SPI_HandleTypeDef hspi1;

void BMI088_GPIO_init(void)
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	  /* GPIO Ports Clock Enable */
	  __HAL_RCC_GPIOB_CLK_ENABLE();
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOC_CLK_ENABLE();
	  __HAL_RCC_GPIOH_CLK_ENABLE();
	  __HAL_RCC_GPIOG_CLK_ENABLE();
	  __HAL_RCC_GPIOF_CLK_ENABLE();

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(RSTN_IST8310_GPIO_Port, RSTN_IST8310_Pin, GPIO_PIN_SET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);

	  /*Configure GPIO pin : PtPin */
	  GPIO_InitStruct.Pin = RSTN_IST8310_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	  HAL_GPIO_Init(RSTN_IST8310_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pins : PGPin PG0 */
	  GPIO_InitStruct.Pin = DRDY_IST8310_Pin|GPIO_PIN_0;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	  /*Configure GPIO pin : PtPin */
	  GPIO_InitStruct.Pin = CS1_ACCEL_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	  HAL_GPIO_Init(CS1_ACCEL_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pins : PCPin PCPin */
	  GPIO_InitStruct.Pin = INT1_ACCEL_Pin|INT1_GYRO_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  /*Configure GPIO pin : PtPin */
	  GPIO_InitStruct.Pin = CS1_GYRO_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	  HAL_GPIO_Init(CS1_GYRO_GPIO_Port, &GPIO_InitStruct);

	  /* EXTI interrupt init*/
	  HAL_NVIC_SetPriority(EXTI0_IRQn, 6, 0);
	  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	  HAL_NVIC_SetPriority(EXTI3_IRQn, 10, 0);
	  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

	  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
	  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
	  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	  MX_SPI1_Init();

}

void BMI088_com_init(void)
{


}

void BMI088_delay_ms(uint16_t ms)
{
    osDelay(ms);
}

void BMI088_delay_us(uint16_t us)
{
    delay_us(us);
}




void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}

void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
}
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}

uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&hspi1, &txdata, &rx_data, 1, 1000);
    return rx_data;
}

