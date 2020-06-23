#include "math.h"
#include "TM1638.h"
//#include "main.h"
//#define VER1

#define ADDR_AUTO  0x40
#define ADDR_FIXED 0x44
#define REG_READ 0x42 	
#define STARTSEGADDR  0xc0//
#define STARTLEDADDR  0xc1 //
#define INDEX_NEGATIVE_SIGN	16
#define INDEX_BLANK     17 // 17 nothing ,0  zero beter for clock

//#define CS_H (GPIOB->BSRR = ((uint16_t)0x1000))
//#define CS_L (GPIOB->BRR = ((uint16_t)0x1000)) 
#define CS_H (GPIOA->BSRR = ((uint16_t)0x0010))
#define CS_L (GPIOA->BRR = ((uint16_t)0x0010))

uint8_t BlankingFlag;
uint8_t SegArray[8];
uint8_t SegArray2[8];
uint8_t  reg_mas[4];
//      A
//     ---
//  F |   | B
//     -G-
//  E |   | C
//     ---
//      D
const uint8_t digitToSegment[] =
{
  0x3f,    // 0 0b00111111
  0x06,    // 1 0b00000110
  0x5b,    // 2 0b01011011
  0x4f,    // 3 0b01001111
  0x66,    // 4 0b01100110
  0x6d,    // 5 0b01101101
  0x7d,    // 6 0b01111101
  0x07,    // 7 0b00000111
  0x7f,    // 8 0b01111111
  0x6f,    // 9 0b01101111
  0x77,    // A 0b01110111
  0x7c,    // b 0b01111100
  0x39,    // C 0b00111001
  0x5e,    // d 0b01011110
  0x79,    // E 0b01111001
  0x71,    // F 0b01110001
  0x40,    // - 0b01000000
  0x00,     // nothing 0b00000000
  0x80     // dot 
  };

const uint8_t digitToSegmentDP[] =
{
  0xbf,    // 0 0b10111111
  0x86,    // 1 0b10000110
  0xdb,    // 2 0b11011011
  0xcf,    // 3 0b11001111
  0xe6,    // 4 0b11100110
  0xed,    // 5 0b11101101
  0xfd,    // 6 0b11111101
  0x87,    // 7 0b10000111
  0xff,    // 8 0b11111111
  0xef,    // 9 0b11101111
  0xf7,    // A 0b11110111
  0xfc,    // b 0b11111100
  0xb9,    // C 0b10111001
  0xde,    // d 0b11011110
  0xf9,    // E 0b11111001
  0xf1,    // F 0b11110001
  0x40,    // - 0b11000000
  0x80,     // nothing 0b00000000
  0x80     // dot
  };

//=========================================================================//
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
//=========================================================================//
void tm1638_Init(uint8_t brightness)
{
	if(brightness>7)return;
	CS_L;
	HAL_Delay(2);
	spi_Send(0x88|brightness);//0x88
	HAL_Delay(2);
	CS_H;
	tm1638_Clear(0);
}

void tm1638_Bright(uint8_t brightness)
{
	if(brightness>7)return;
	CS_L;
	HAL_Delay(1);
	spi_Send(0x88|brightness);//0x88
	HAL_Delay(1);
	CS_H;
}
//++++++++++++++++++++++управление одним светодиодом+++++++++++++++++++++++//
//=========================================================================//
void led_Set(uint8_t led_Num, uint8_t state){//led_Num 1-8
	if(led_Num>8 || led_Num<1)return;
	CS_L;
	spi_Send(ADDR_FIXED);  
	CS_H;

	CS_L;
	spi_Send(STARTLEDADDR+(led_Num-1)*2);
	spi_Send(state);
	CS_H;
}
//=========================================================================//
//+++++++++++++++++++++++управление всеми светодиодами+++++++++++++++++++++//
//=========================================================================//
void leds_Set(uint8_t state){//+
	uint8_t led_Num = 0;
	CS_L;
	spi_Send(ADDR_FIXED);  
	CS_H;
 for(;led_Num<8;led_Num++){
	CS_L;
	spi_Send(STARTLEDADDR+led_Num*2);
	spi_Send(state&0x01);
	state >>= 1; 
	CS_H;
 }
}
//=========================================================================//
//++++++++++++++++++++++++отображает i-ый сегмент++++++++++++++++++++++++++//
//=========================================================================//
void segmentN_Set(uint8_t seg_Num, uint8_t data){//seg_Num 1-8;
	if(seg_Num>8 || seg_Num<1)return;
	seg_Num=seg_Num-1;
	data=digitToSegment[data];
	CS_L;
	spi_Send(ADDR_FIXED); 
	CS_H;

	CS_L;
	spi_Send(STARTSEGADDR+seg_Num*2);//0,2,4,6,8,10....
	spi_Send(data);
	CS_H;
}

//=========================================================================//
//+++++++++++++++++++++++++отображает массив+++++++++++++++++++++++++++++++//
//=========================================================================//
void segments_Set(uint8_t start_Seg, uint8_t stop_Seg, uint8_t data[]){//+
	uint8_t i=0;
	CS_L;
	spi_Send(ADDR_FIXED); 
	CS_H;

		for(;start_Seg < stop_Seg;start_Seg++,i++){
			CS_L;
			spi_Send(STARTSEGADDR+start_Seg*2);
			spi_Send(data[i]);
			CS_H;
		}
}
//=========================================================================//
//+++++++++++++++++++++читаем состояние кнопок+++++++++++++++++++++++++++++//
//=========================================================================//
uint8_t tm1638_ReadKeys(){//+
	uint8_t keys = 0; //reg_mas[4];
  uint8_t i=0;
	
	CS_L;
	spi_Read_Reg(reg_mas);//
	CS_H;
  for(;i<4;i++) keys |= (reg_mas[i]&0x11)<<i;
	return keys;
}

void tm1638_Clear(uint8_t parameter){//0-all;1-segments;2-led
	uint8_t i=0;

	CS_L;
	if(!parameter)spi_Send(ADDR_AUTO);
	   else spi_Send(ADDR_FIXED);
	CS_H;

		switch(parameter){
			case 0://all
				CS_L;
				spi_Send(STARTSEGADDR);
	     	for(; i < 16; i ++) spi_Send(0);
			  CS_H;
			break;
		  case 1://segments
				for(; i < 8; i ++){
		    CS_L;
		    spi_Send(STARTSEGADDR+i*2);
		    spi_Send(0);
		    CS_H;
	     }
			break;
			case 2://led
				for(;i<8;i++){
	         CS_L;
	         spi_Send(STARTLEDADDR+i*2);
	         spi_Send(0);
	         CS_H;
     }
			break;
		}

}

//=========================================================================//
//|
//=========================================================================//
void tm1638_Digit(int digit,uint8_t pos)//
{
	uint8_t i=0;
	uint8_t digit_starts=0;

	if(digit < 0)
	{
		SegArray[0] = INDEX_NEGATIVE_SIGN;
		digit = (digit & 0x7fffffff);
		SegArray[1] = digit/10000;
		digit %= 10000;
		SegArray[2] = digit/1000;
		digit %= 1000;
		SegArray[3] = digit/100;
		digit %= 100;
		SegArray[4] = digit / 10;
		SegArray[5] = digit % 10;
		if(BlankingFlag)
		{
			if(SegArray[1] == 0)
			{
				SegArray[1] = INDEX_BLANK;
				if(SegArray[2] == 0){
					SegArray[2] = INDEX_BLANK;
				  if(SegArray[3] == 0){
					   SegArray[3] = INDEX_BLANK;
						 if(SegArray[4] == 0){
					   SegArray[4] = INDEX_BLANK;
							 if(SegArray[5] == 0){
					        SegArray[5] = INDEX_BLANK;
							 }
						 }
					}
				}
			}
		}
	}
	else
	{
		SegArray[0] = digit/10000;
		digit %= 10000;
		SegArray[1] = digit/1000;
		digit %= 1000;
		SegArray[2] = digit/100;
		digit %= 100;
		SegArray[3] = digit/10;
		SegArray[4] = digit % 10;
		if(BlankingFlag)
		{
			if(SegArray[0] == 0)
			{
				SegArray[0] = INDEX_BLANK;
				if(SegArray[1] == 0)
				{
					SegArray[1] = INDEX_BLANK;
					if(SegArray[2] == 0){
					   SegArray[2] = INDEX_BLANK;
						 	if(SegArray[3] == 0){
								 SegArray[3] = INDEX_BLANK;
//								 if(SegArray[4] == 0){
//								   SegArray[4] = INDEX_BLANK;
//								 }
							}
					}
				}
			}
		}
	}
	BlankingFlag = 1;

	while(SegArray[i]==INDEX_BLANK)i++;
	digit_starts=(i);
	for(;i<MAX_SEGMENTS;i++){
	SegArray[i-digit_starts]=digitToSegment[SegArray[i]];
	SegArray[i]=0;
	}
	segments_Set(digit_starts+pos-1,MAX_SEGMENTS,SegArray);//0-8
}

void tm1638_FloatDigit(float digit,uint8_t num_signes) //
{
	int a = 10; // ��� ������ ���� �� ����
	uint8_t digit_starts=0;
	uint8_t i;

	for(i = 1; i < num_signes; i++) // ������� ���
	{
		a= a*10;
	}
	a = digit * a;

	{
		SegArray[0] = a/100000;
		a %= 100000;
		SegArray[1] = a/10000;
		a %= 10000;
		SegArray[2] = a/1000;
		a %= 1000;
		SegArray[3] = a/100;
		a %= 100;
		SegArray[4] = a / 10;
		SegArray[5] = a % 10;

	if(BlankingFlag)
	{
		if (SegArray[0] == 0)
		{
			SegArray[0] = INDEX_BLANK;
			if (SegArray[1] == 0)
			{
				SegArray[1] = INDEX_BLANK;
				if ( (SegArray[2] == 0) & (num_signes != 3) )
				{
					SegArray[2] = INDEX_BLANK;
					if ( (SegArray[3] == 0) & (num_signes != 2))
					{
						SegArray[3] = INDEX_BLANK;
						if ( (SegArray[4] == 0) & (num_signes != 1))
						{
							SegArray[4] = INDEX_BLANK;
						}
					}
				}
			}
		}
	}
	}

	BlankingFlag = 1;

	i = 0;
	while (SegArray[i]==INDEX_BLANK) i++;
	digit_starts = i;
	for(; i<MAX_SEGMENTS; i++)
	{
		if ( ((MAX_SEGMENTS-1)-i) == num_signes )
		{
			SegArray[i-digit_starts] = digitToSegmentDP[SegArray[i]];
		}
		else
		{
			SegArray[i-digit_starts]=digitToSegment[SegArray[i]];
		}
		SegArray[i]=0;
	}
#ifdef VER1
		for(i=0;i<3;i++)
		{
			uint8_t a = SegArray[i+3];
			SegArray[i+3] = SegArray[i];
			SegArray[i] = a;
		}
#endif
		segments_Set(digit_starts,MAX_SEGMENTS,SegArray);//0-8
}
