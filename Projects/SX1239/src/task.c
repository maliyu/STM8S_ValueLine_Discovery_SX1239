/******************************************************************************
 * Project        : STM8S_Discovery_SX1278_SFM1L
 * File           : task.c
 * Copyright      : 2014 Yosun Singapore Pte Ltd
 ******************************************************************************
  Change History:

    Version 1.0 - Sep 2014
    > Initial revision

******************************************************************************/
#include <string.h>
#include "stm8s.h"
#include "sx1239.h"
#include "task.h"
#include "board.h"

static uint8_t RFbuffer[RF_BUFFER_SIZE]; // RF buffer
static volatile uint8_t RFbufferSize; // RF buffer size

void task_exec(void)
{
	uint8_t returnCode = ERROR;

	ReceiveRfFrame(RFbuffer, (uint8_t *)&RFbufferSize, &returnCode);

	if(OK == returnCode)
	{
		if(RFbufferSize > 0)
		{
			RFbuffer[RFbufferSize] = '\0';

			if(strcmp(RFbuffer, "PING") == 0)
			{
				
			}
		}
	}
}

