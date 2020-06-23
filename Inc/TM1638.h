#include "stm32f1xx_hal.h"
#include "main.h"

#ifndef TM1638_H_
#define TM1638_H_
#define LED_1 1<<0
#define LED_2 1<<1
#define LED_3 1<<2
#define LED_4 1<<3
#define LED_5 1<<4
#define LED_6 1<<5
#define LED_7 1<<6
#define LED_8 1<<7
#define ALIGMENT 245
#define MAX_SEGMENTS 5
#define DELAY1_BRIGHT 5

void tm1638_Init(uint8_t brightness);
void led_Set(uint8_t led_Num, uint8_t state);
void leds_Set(uint8_t state);
void segmentN_Set(uint8_t seg_Num, uint8_t data);
void segments_Set(uint8_t start_Seg, uint8_t stop_Seg, uint8_t data[]);
void tm1638_Digit(int digit,uint8_t pos);
//void tm1638_FloatDigit(float digit,uint8_t num_signes, uint8_t pos, uint8_t aligment);
void tm1638_FloatDigit(float digit,uint8_t num_signes);
void tm1638_Bright(uint8_t brightness);
void tm1638_Clear(uint8_t parameter);
uint8_t tm1638_ReadKeys(void);


#endif 
