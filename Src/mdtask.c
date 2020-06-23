#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#include "mb.h"
#include "mbport.h"

#define REG_INPUT_START 1000
#define REG_INPUT_NREGS 2
#define REG_HOLDING_START 1000
#define REG_HOLDING_NREGS 2

static USHORT usRegInputStart = REG_INPUT_START;
static USHORT usRegInputBuf[REG_INPUT_NREGS];
static USHORT usRegHoldingStart = REG_HOLDING_START;
static USHORT usRegHoldingBuf[REG_HOLDING_NREGS];

osSemaphoreId xReleaseLockHandle = NULL;

void ModbusRTUTask(void const * argument)
{ 
  usRegInputBuf[0] = 01;
  usRegInputBuf[1] = 12;

  usRegHoldingBuf[0] = 01;
  usRegHoldingBuf[1] = 12;

//  eMBErrorCode eStatus = eMBInit( MB_RTU, 3, 2, 256000, MB_PAR_NONE );    // eMode,ucSlaveAddress,ucPort,ulBaudRate,eParity
  eMBInit( MB_RTU, 1, 3, 115200, MB_PAR_NONE );    // eMode,ucSlaveAddress,ucPort,ulBaudRate,eParity
  eMBEnable();
  
  while(1) {
    eMBPoll();
  }
}

eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if( ( usAddress >= REG_INPUT_START )
        && ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegInputStart );
        while( usNRegs > 0 )
        {
            *pucRegBuffer++ =
                ( unsigned char )( usRegInputBuf[iRegIndex] >> 8 );
            *pucRegBuffer++ =
                ( unsigned char )( usRegInputBuf[iRegIndex] & 0xFF );
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;			
    }

    return eStatus;
}

eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs,
                 eMBRegisterMode eMode )
{
    eMBErrorCode  eStatus = MB_ENOERR;
    int           iRegIndex;

    if( ( usAddress >= REG_HOLDING_START )
        && ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegHoldingStart );
        while( usNRegs > 0 )
        {
            switch (eMode)
            {
                case MB_REG_READ:
                    if (iRegIndex == 1)
                    {
                      usRegHoldingBuf[1] = 0xF0;
                      if ( HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET ) { usRegHoldingBuf[1] |= GPIO_PIN_0; }
                      if ( HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_SET ) { usRegHoldingBuf[1] |= GPIO_PIN_1; }
                      if ( HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET ) { usRegHoldingBuf[1] |= GPIO_PIN_2; }
                      if ( HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_SET ) { usRegHoldingBuf[1] |= GPIO_PIN_3; }
                      if ( HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_SET ) { usRegHoldingBuf[1] |= GPIO_PIN_4; }
                      if ( HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_SET ) { usRegHoldingBuf[1] |= GPIO_PIN_5; }
                      if ( HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET ) { usRegHoldingBuf[1] |= GPIO_PIN_6; }
                      if ( HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET ) { usRegHoldingBuf[1] |= GPIO_PIN_7; }
                    }
                    *pucRegBuffer++ =
                        ( unsigned char )( usRegHoldingBuf[iRegIndex] >> 8 );
                    *pucRegBuffer++ =
                        ( unsigned char )( usRegHoldingBuf[iRegIndex] & 0xFF );
                    break;
                case MB_REG_WRITE:
                  usRegHoldingBuf[iRegIndex] = ( unsigned char ) *pucRegBuffer++ << 8;
                  usRegHoldingBuf[iRegIndex] = ( unsigned char ) *pucRegBuffer++ & 0xFF;
                  if (iRegIndex == 0)
                  {
                    if ( (usRegHoldingBuf[0] & GPIO_PIN_0) != 0x00 ) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
                    if ( (usRegHoldingBuf[0] & GPIO_PIN_1) != 0x00 ) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
                    if ( (usRegHoldingBuf[0] & GPIO_PIN_2) != 0x00 ) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
                    if ( (usRegHoldingBuf[0] & GPIO_PIN_3) != 0x00 ) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
                    if ( (usRegHoldingBuf[0] & GPIO_PIN_4) != 0x00 ) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
                    if ( (usRegHoldingBuf[0] & GPIO_PIN_5) != 0x00 ) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
                    if ( (usRegHoldingBuf[0] & GPIO_PIN_6) != 0x00 ) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
                    if ( (usRegHoldingBuf[0] & GPIO_PIN_7) != 0x00 ) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
                    xSemaphoreGive( xReleaseLockHandle );
                  }
                  break;
            }
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
      eStatus = MB_ENOREG;
    }
    return eStatus;
}


eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils,
               eMBRegisterMode eMode )
{
    return MB_ENOREG;
}

eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
    return MB_ENOREG;
}
