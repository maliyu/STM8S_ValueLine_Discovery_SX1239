/******************************************************************************
 * Project        : STM8S_Discovery_SX1278_SFM1L
 * File           : board.c
 * Copyright      : 2014 Yosun Singapore Pte Ltd
 ******************************************************************************
  Change History:

    Version 1.0 - Sep 2014
    > Initial revision

******************************************************************************/
#include "stm8s.h"
#include "board.h"

#define ISUNSIGNED(a) (a>0 && ~a>0)

static void SPI_SoftInit(void)
{	
	/* SPI init */
	/* soft SPI due to issue on hardware SPI */
	/* Configure PE5 (SPI_NSS) as output push-pull low */
	GPIO_Init(SPI_NSS_PORT, SPI_NSS_PIN, GPIO_MODE_OUT_PP_HIGH_FAST);
	/* Configure PC5 (SPI_SCK) as output push-pull low */
	GPIO_Init(SPI_SCK_PORT, SPI_SCK_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
	/* Configure PC6 (SPI_MOSI) as output push-pull low */
	GPIO_Init(SPI_MOSI_PORT, SPI_MOSI_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
	/* Configure PC7 (SPI_MISO) as input pull-up */
	GPIO_Init(SPI_MISO_PORT, SPI_MISO_PIN, GPIO_MODE_IN_PU_NO_IT);
}

static void TIM1_Init(void)
{
	TIM1_TimeBaseInit(15, TIM1_COUNTERMODE_UP, 999, 0);
	TIM1_SetCounter(0);
	TIM1_ARRPreloadConfig(DISABLE);
	TIM1_ITConfig(TIM1_IT_UPDATE, ENABLE);
	TIM1_Cmd(DISABLE);
}

void Board_Init(void)
{
  /* fmaster = 16MHz internal clock*/
  /* fCPU = fmaster */
  //CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV1);
  //CLK_SYSCLKConfig(CLK_PRESCALER_CPUDIV1);
  CLK->CKDIVR = 0x00;
  
  /* Configure PD0 (LED1) as output push-pull low (led switched on) */
  //GPIO_Init(GPIOD, GPIO_PIN_0, GPIO_MODE_OUT_PP_HIGH_FAST);
  GPIO_Init(GPIOD, GPIO_PIN_0, GPIO_MODE_OUT_PP_LOW_FAST);

  /* Configure PC3 as SX1239 DIO0 interrupt pin */
  GPIO_Init(SX1239_DIO0_PORT, SX1239_DIO0_PIN, GPIO_MODE_IN_PU_IT);

  TIM1_Init();

  SPI_SoftInit();
}

void SPI_write(uint8_t addr, uint8_t data)
{
  uint8_t tmp = addr | 0x80;
  uint8_t i;
 
    GPIO_WriteLow(SPI_NSS_PORT, SPI_NSS_PIN);
    GPIO_WriteLow(SPI_SCK_PORT, SPI_SCK_PIN);
    
    for(i=8;i!=0;i--)
    {
      GPIO_WriteLow(SPI_SCK_PORT, SPI_SCK_PIN);
      if(tmp & 0x80)
      {
        GPIO_WriteHigh(SPI_MOSI_PORT, SPI_MOSI_PIN);
      }
      else
      {
        GPIO_WriteLow(SPI_MOSI_PORT, SPI_MOSI_PIN);
      }
      asm("nop");
      asm("nop");
      GPIO_WriteHigh(SPI_SCK_PORT, SPI_SCK_PIN);
      asm("nop");
      tmp <<= 1;
    }
    
    GPIO_WriteLow(SPI_SCK_PORT, SPI_SCK_PIN);
    GPIO_WriteHigh(SPI_MOSI_PORT, SPI_MOSI_PIN);
    
      tmp = data;
      
      GPIO_WriteLow(SPI_NSS_PORT, SPI_NSS_PIN);
      GPIO_WriteLow(SPI_SCK_PORT, SPI_SCK_PIN);
      
      for(i=8;i!=0;i--)
      {
        GPIO_WriteLow(SPI_SCK_PORT, SPI_SCK_PIN);
        if(tmp & 0x80)
        {
          GPIO_WriteHigh(SPI_MOSI_PORT, SPI_MOSI_PIN);
        }
        else
        {
          GPIO_WriteLow(SPI_MOSI_PORT, SPI_MOSI_PIN);
        }
        asm("nop");
        asm("nop");
        GPIO_WriteHigh(SPI_SCK_PORT, SPI_SCK_PIN);
        asm("nop");
        tmp <<= 1;
      }
      
      GPIO_WriteLow(SPI_SCK_PORT, SPI_SCK_PIN);
      GPIO_WriteHigh(SPI_MOSI_PORT, SPI_MOSI_PIN);

    GPIO_WriteHigh(SPI_NSS_PORT, SPI_NSS_PIN);
}

uint8_t SPI_read(uint8_t addr)
{
	uint8_t tmp = addr;
	uint8_t i;
  
    GPIO_WriteLow(SPI_NSS_PORT, SPI_NSS_PIN);
    GPIO_WriteLow(SPI_SCK_PORT, SPI_SCK_PIN);
    
    for(i=8;i!=0;i--)
    {
      GPIO_WriteLow(SPI_SCK_PORT, SPI_SCK_PIN);
      if(tmp & 0x80)
      {
        GPIO_WriteHigh(SPI_MOSI_PORT, SPI_MOSI_PIN);
      }
      else
      {
        GPIO_WriteLow(SPI_MOSI_PORT, SPI_MOSI_PIN);
      }
      asm("nop");
      asm("nop");
      GPIO_WriteHigh(SPI_SCK_PORT, SPI_SCK_PIN);
      asm("nop");
      tmp <<= 1;
    }
    
    GPIO_WriteLow(SPI_SCK_PORT, SPI_SCK_PIN);
    GPIO_WriteHigh(SPI_MOSI_PORT, SPI_MOSI_PIN);
    
	GPIO_WriteLow(SPI_NSS_PORT, SPI_NSS_PIN);
	GPIO_WriteHigh(SPI_MOSI_PORT, SPI_MOSI_PIN);
      
	tmp = 0;
	for(i=8;i!=0;i--)
	{
		GPIO_WriteLow(SPI_SCK_PORT, SPI_SCK_PIN);
        asm("nop");
        asm("nop");
        tmp <<= 1;
        GPIO_WriteHigh(SPI_SCK_PORT, SPI_SCK_PIN);
        if(GPIO_ReadInputPin(SPI_MISO_PORT, SPI_MISO_PIN))
        {
          tmp |= 0x01;
        }
        else
        {
          tmp |= 0x00;
        }
	}
	GPIO_WriteLow(SPI_SCK_PORT, SPI_SCK_PIN);
   
    GPIO_WriteHigh(SPI_NSS_PORT, SPI_NSS_PIN);

	return tmp;
}