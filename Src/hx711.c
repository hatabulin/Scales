#include "hx711.h"

extern uint8_t us_const1;

void HX711_Init(HX711 data) {
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = data.pinSck;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(data.gpioSck, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = data.pinData;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(data.gpioData, &GPIO_InitStruct);


	// Enable Timer 4 for precise wait
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	TIM3->CR1 |= TIM_CR1_ARPE | TIM_CR1_DIR;
	// Prescale by 1 to get 10MHz clock
	TIM4->PSC = 3;


  // Set the clock pin high (IDLE)
	HAL_GPIO_WritePin(data.gpioSck, data.pinSck, GPIO_PIN_SET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(data.gpioData, data.pinSck, GPIO_PIN_RESET);
}

int HX711_AverageValue(HX711 data,  uint8_t times)
{
    int sum = 0;
    for (int i = 0; i < times; i++) sum += HX711_Value(data);// - data.offset);
    return sum / times;
}

int HX711_Value(HX711 data) {
    long buffer = 0;
/*
    uint16_t timeout = 10000;
    while ((HAL_GPIO_ReadPin(data.gpioData, data.pinData)==1) & (timeout >0))
    {
    	timeout--;
    }
    if (timeout != 1) return (0); // fault
 */
    HAL_GPIO_WritePin(data.gpioData, data.pinData,SET);
    DWT_Delay(1);
    HAL_GPIO_WritePin(data.gpioSck, data.pinSck, GPIO_PIN_RESET);
    while (HAL_GPIO_ReadPin(data.gpioData, data.pinData)==1) {};
    DWT_Delay(1);
    for (uint8_t i = 0; i < 24; i++) {
    	HAL_GPIO_WritePin(data.gpioSck, data.pinSck, GPIO_PIN_SET);
    	DWT_Delay(1);
    	buffer = buffer << 1 ;
    	HAL_GPIO_WritePin(data.gpioSck, data.pinSck, GPIO_PIN_RESET);

        if (HAL_GPIO_ReadPin(data.gpioData, data.pinData)) buffer ++;
//        HAL_GPIO_WritePin(data.gpioSck, data.pinSck, GPIO_PIN_RESET);
    }
    for (int i = 0; i < data.gain; i++) {
    	HAL_GPIO_WritePin(data.gpioSck, data.pinSck, GPIO_PIN_SET);
    	DWT_Delay(us_const1);
    	HAL_GPIO_WritePin(data.gpioSck, data.pinSck, GPIO_PIN_RESET);
    }
    buffer = buffer ^ 0x800000;

    return buffer;
}

HX711 HX711_Tare(HX711 data, uint8_t times) {
    int sum = HX711_AverageValue(data, times);
    data.offset = sum;
    return data;
}

