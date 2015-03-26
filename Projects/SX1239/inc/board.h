/******************************************************************************
 * Project        : STM8S_Discovery_SX1278_SFM1L
 * File           : board.h
 * Copyright      : 2014 Yosun Singapore Pte Ltd
 ******************************************************************************
  Change History:

    Version 1.0 - Sep 2014
    > Initial revision

******************************************************************************/
#ifndef _BOARD_H_
#define _BOARD_H_

#define REMOTE_MAX_NUMBER       10
#define VERSION_MAX_SIZE        5
#define FIRMWARE_VERSION_ADDRESS        0x4000
#define DEVICE_PARAMETERS_ADDRESS       (FIRMWARE_VERSION_ADDRESS + VERSION_MAX_SIZE)

#define SPI_SCK_PORT    GPIOC
#define SPI_SCK_PIN     GPIO_PIN_5
#define SPI_MOSI_PORT   GPIOC
#define SPI_MOSI_PIN    GPIO_PIN_6
#define SPI_MISO_PORT   GPIOC
#define SPI_MISO_PIN    GPIO_PIN_7
#define SPI_NSS_PORT    GPIOA
#define SPI_NSS_PIN     GPIO_PIN_3 

#define SX1239_DIO0_PORT	GPIOC
#define SX1239_DIO0_PIN	GPIO_PIN_3

typedef struct t_DeviceParameters
{
  unsigned char hostID;
  unsigned char remoteID[REMOTE_MAX_NUMBER];
}s_DeviceParameters;

void Board_Init(void);
unsigned char SPI_read(unsigned char addr);
void SPI_write(unsigned char addr, unsigned char data);
#endif /* _BOARD_H_ */
