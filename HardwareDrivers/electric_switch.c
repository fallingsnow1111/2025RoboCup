#include "electric_switch.h"

uint8_t ready_flag = 0;
void switch_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pins : PB12 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void IsReady(void){
	while(1){
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15)==0)
		{
//			HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_9);
			ready_flag = 1;
			buzzer_complete_flag = 1;
			break;
		}
		vTaskDelay(20);
	}
}
