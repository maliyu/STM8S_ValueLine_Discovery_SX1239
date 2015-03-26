/*******************************************************************
** File        : SX1231driver.c                                   **
********************************************************************
**                                                                **
** Version     : V 2.0                                            **
**                                                                **
** Written by  : Chaouki ROUAISSIA                                **
**                                                                **
** Date        : 07-10-2009                                       **
**                                                                **
** Project     : API-1231                                         **
**                                                                **
********************************************************************
** Description : SX1231 transceiver drivers Implementation for the**
**               XE8000 family products !!! Packet mode !!!       **
*******************************************************************/

/*******************************************************************
** Include files                                                  **
*******************************************************************/
#include "stm8s.h"
#include "board.h"
#include "SX1239.h"

/*******************************************************************
** Global variables                                               **
*******************************************************************/
static   uint8_t RFState = RF_STOP;     // RF state machine
static   uint8_t *pRFFrame;             // Pointer to the RF frame
static   uint8_t RFFramePos;            // RF payload current position
static   uint8_t RFFrameSize;           // RF payload size
static  uint16_t ByteCounter = 0;       // RF payload byte counter
static   uint8_t PreMode = RF_STANDBY;  // Previous chip operating mode
static   uint8_t SyncSize = 8;          // Size of sync word
static   uint8_t SyncValue[8];          // Value of sync word
static  uint32_t RFFrameTimeOut = RF_FRAME_TIMEOUT(1200); // Reception counter value (full frame timeout generation)


uint16_t RegistersCfg[] = { // SX1231 configuration registers values
	DEF_FIFO, // Left for convenience, not to be changed
	DEF_OPMODE | RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY,                       
	DEF_DATAMODUL | RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00,                    
	DEF_BITRATEMSB | RF_BITRATEMSB_4800,                   
	DEF_BITRATELSB | RF_BITRATELSB_4800,                   
	DEF_FDEVMSB | RF_FDEVMSB_5000,                       
	DEF_FDEVLSB | RF_FDEVLSB_5000,                      
	DEF_FRFMSB | RF_FRFMSB_865,                       
	DEF_FRFMID | RF_FRFMID_865,                       
	DEF_FRFLSB | RF_FRFLSB_865,                       
	DEF_OSC1,                         
	DEF_OSC2,                         
	DEF_LOWBAT | RF_LOWBAT_OFF | RF_LOWBAT_TRIM_1835,                                               
	DEF_LISTEN1 | RF_LISTEN1_RESOL_4100 | RF_LISTEN1_CRITERIA_RSSI | RF_LISTEN1_END_01,                      
	DEF_LISTEN2 | RF_LISTEN2_COEFIDLE_VALUE,                      
	DEF_LISTEN3 | RF_LISTEN3_COEFRX_VALUE, 
	DEF_VERSION, 			// Read Only                      

	DEF_PALEVEL | RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111,                          
	DEF_PARAMP | RF_PARAMP_40,                          
	DEF_OCP | RF_OCP_ON | RF_OCP_TRIM_100,                           

	DEF_AGCREF | RF_AGCREF_AUTO_ON | RF_AGCREF_LEVEL_MINUS80,                          
	DEF_AGCTHRESH1 | RF_AGCTHRESH1_SNRMARGIN_101 | RF_AGCTHRESH1_STEP1_16,                          
	DEF_AGCTHRESH2 | RF_AGCTHRESH2_STEP2_3 | RF_AGCTHRESH2_STEP3_11,                          
	DEF_AGCTHRESH3 | RF_AGCTHRESH3_STEP4_9 | RF_AGCTHRESH3_STEP5_11,                          
	DEF_LNA | RF_LNA_ZIN_200 | RF_LNA_LOWPOWER_OFF | RF_LNA_GAINSELECT_AUTO,                           
	DEF_RXBW | RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_5,                          
	DEF_AFCBW | RF_AFCBW_DCCFREQAFC_100 | RF_AFCBW_MANTAFC_20 | RF_AFCBW_EXPAFC_3,                        
	DEF_OOKPEAK | RF_OOKPEAK_THRESHTYPE_PEAK | RF_OOKPEAK_PEAKTHRESHSTEP_000 | RF_OOKPEAK_PEAKTHRESHDEC_000,                         
	DEF_OOKAVG | RF_OOKAVG_AVERAGETHRESHFILT_10,								  
	DEF_OOKFIX | RF_OOKFIX_FIXEDTHRESH_VALUE,								  
	DEF_AFCFEI | RF_AFCFEI_AFCAUTOCLEAR_OFF | RF_AFCFEI_AFCAUTO_OFF,							  
	DEF_AFCMSB, 			// Read Only							  
	DEF_AFCLSB, 			// Read Only							  
	DEF_FEIMSB, 			// Read Only							  
	DEF_FEILSB, 			// Read Only							  
	DEF_RSSICONFIG | RF_RSSI_FASTRX_OFF,						  
	DEF_RSSIVALUE,  		// Read Only						  

	DEF_DIOMAPPING1 | RF_DIOMAPPING1_DIO0_00 | RF_DIOMAPPING1_DIO1_00 | RF_DIOMAPPING1_DIO2_00 | RF_DIOMAPPING1_DIO3_00,
	DEF_DIOMAPPING2 | RF_DIOMAPPING2_DIO4_00 | RF_DIOMAPPING2_DIO5_01 | RF_DIOMAPPING2_CLKOUT_OFF,						  
	DEF_IRQFLAGS1,
	DEF_IRQFLAGS2,								  							  
	DEF_RSSITHRESH | 228,	// Must be set to (-Sensitivity x 2)						  
	DEF_RXTIMEOUT1 | RF_RXTIMEOUT1_RXSTART_VALUE,						  
	DEF_RXTIMEOUT2 | RF_RXTIMEOUT2_RSSITHRESH_VALUE,						  
 
	DEF_PREAMBLEMSB | RF_PREAMBLESIZE_MSB_VALUE,						  		
	DEF_PREAMBLELSB | RF_PREAMBLESIZE_LSB_VALUE,						  		
	DEF_SYNCCONFIG | RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_4 | RF_SYNC_TOL_0,		   		     
	DEF_SYNCVALUE1 | 0x69,						  
	DEF_SYNCVALUE2 | 0x81,						  
	DEF_SYNCVALUE3 | 0x7E,						  
	DEF_SYNCVALUE4 | 0x96,						  
	DEF_SYNCVALUE5 | RF_SYNC_BYTE5_VALUE,						  
	DEF_SYNCVALUE6 | RF_SYNC_BYTE6_VALUE,						  
	DEF_SYNCVALUE7 | RF_SYNC_BYTE7_VALUE,						  
	DEF_SYNCVALUE8 | RF_SYNC_BYTE8_VALUE,						  
	DEF_PACKETCONFIG1 | RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF,					  
	DEF_PAYLOADLENGTH | 255,					  		
	DEF_NODEADRS | RF_NODEADDRESS_VALUE,							  		
	DEF_BROADCASTADRS | RF_BROADCASTADDRESS_VALUE,					  		
	DEF_AUTOMODES | RF_AUTOMODES_ENTER_OFF | RF_AUTOMODES_EXIT_OFF | RF_AUTOMODES_INTERMEDIATE_SLEEP,						  		
	DEF_FIFOTHRESH | RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE,						  
	DEF_PACKETCONFIG2 | RF_PACKET2_RXRESTARTDELAY_1BIT | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF,					  
	DEF_AESKEY1 | RF_AESKEY1_VALUE,							  
	DEF_AESKEY2 | RF_AESKEY2_VALUE,							  
	DEF_AESKEY3 | RF_AESKEY3_VALUE,							  
	DEF_AESKEY4 | RF_AESKEY4_VALUE,							  
	DEF_AESKEY5 | RF_AESKEY5_VALUE,							  
	DEF_AESKEY6 | RF_AESKEY6_VALUE,							  
	DEF_AESKEY7 | RF_AESKEY7_VALUE,							  
	DEF_AESKEY8 | RF_AESKEY8_VALUE,							  
	DEF_AESKEY9 | RF_AESKEY9_VALUE,							  
	DEF_AESKEY10 | RF_AESKEY10_VALUE,							  
	DEF_AESKEY11 | RF_AESKEY11_VALUE,							  
	DEF_AESKEY12 | RF_AESKEY12_VALUE,							  
	DEF_AESKEY13 | RF_AESKEY13_VALUE,							  
	DEF_AESKEY14 | RF_AESKEY14_VALUE,							  
	DEF_AESKEY15 | RF_AESKEY15_VALUE,							  
	DEF_AESKEY16 | RF_AESKEY16_VALUE,							  
	
	DEF_TEMP1 | RF_TEMP1_ADCLOWPOWER_ON,					  		      
	DEF_TEMP2							     
};

/*******************************************************************
** Configuration functions                                        **
*******************************************************************/

/*******************************************************************
** InitRFChip : This routine initializes the RFChip registers     **
**              Using Pre Initialized variables                   **
********************************************************************
** In  : -                                                        **
** Out : -                                                        **
*******************************************************************/
void InitRFChip (void){
    uint16_t i;
		 
	/////// RC CALIBRATION (Once at POR) ///////
	SetRFMode(RF_STANDBY);
	WriteRegister(0x57, 0x80); 
	WriteRegister(REG_OSC1, ReadRegister(REG_OSC1) | RF_OSC1_RCCAL_START); 
	while ((ReadRegister(REG_OSC1) & RF_OSC1_RCCAL_DONE) == 0x00); 
 	WriteRegister(REG_OSC1, ReadRegister(REG_OSC1) | RF_OSC1_RCCAL_START); 
	while ((ReadRegister(REG_OSC1) & RF_OSC1_RCCAL_DONE) == 0x00);
	WriteRegister(0x57, 0x00);
	////////////////////////////////////////////
	 
    for(i = 1; i <= REG_TEMP2; i++){
    	  WriteRegister(i, RegistersCfg[i]);
    }

    SyncSize = ((RegistersCfg[REG_SYNCCONFIG] >> 3) & 0x07) + 1;
    for(i = 0; i < SyncSize; i++){
        SyncValue[i] = RegistersCfg[REG_SYNCVALUE1 + i];
    }

    RFFrameTimeOut = RF_FRAME_TIMEOUT(1200); // Worst case bitrate

    SetRFMode(RF_SLEEP);
}

/*******************************************************************
** SetRFMode : Sets the SX1231 operating mode                     **
********************************************************************
** In  : mode                                                     **
** Out : -                                                        **
*******************************************************************/
void SetRFMode(uint8_t mode){
    if(mode != PreMode){
        if(mode == RF_TRANSMITTER){
        		WriteRegister(REG_OPMODE, (RegistersCfg[REG_OPMODE] & 0xE3) | RF_TRANSMITTER);        
        		while ((ReadRegister(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady
        		PreMode = RF_TRANSMITTER;
        }
        
        else if(mode == RF_RECEIVER){
        		WriteRegister(REG_OPMODE, (RegistersCfg[REG_OPMODE] & 0xE3) | RF_RECEIVER);        
        		while ((ReadRegister(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady
        		PreMode = RF_RECEIVER;
        }
        
        else if(mode == RF_SYNTHESIZER){
        		WriteRegister(REG_OPMODE, (RegistersCfg[REG_OPMODE] & 0xE3) | RF_SYNTHESIZER);        
        		while ((ReadRegister(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady
        		PreMode = RF_SYNTHESIZER;
        }
        
        else if(mode == RF_STANDBY){
        		WriteRegister(REG_OPMODE, (RegistersCfg[REG_OPMODE] & 0xE3) | RF_STANDBY);        
        		while ((ReadRegister(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady
        		PreMode = RF_STANDBY;
        }
        
        else {// mode == RF_SLEEP
        		WriteRegister(REG_OPMODE, (RegistersCfg[REG_OPMODE] & 0xE3) | RF_SLEEP);        
        		while ((ReadRegister(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady
        		PreMode = RF_SLEEP;
        }
    }
}

/*******************************************************************
** WriteRegister : Writes the register value at the given address **
**                  on the SX1231                                 **
********************************************************************
** In  : address, value                                           **
** Out : -                                                        **
*******************************************************************/
void WriteRegister(uint8_t address, uint16_t value){
	SPI_write(address, (uint8_t)value);
}

/*******************************************************************
** ReadRegister : Reads the register value at the given address on**
**                the SX1231                                      **
********************************************************************
** In  : address                                                  **
** Out : value                                                    **
*******************************************************************/
uint16_t ReadRegister(uint8_t address){
    return (uint16_t)SPI_read(address);
}


/*******************************************************************
** Communication functions                                        **
*******************************************************************/

/*******************************************************************
** ReceiveRfFrame : Receives a RF frame                           **
********************************************************************
** In  : -                                                        **
** Out : *buffer, size, *pReturnCode                              **
*******************************************************************/
void ReceiveRfFrame(uint8_t *buffer, uint8_t *size, uint8_t *pReturnCode){
    
       uint8_t TempRFState;
       
       *pReturnCode = RX_RUNNING;

       TempRFState = RFState; 
	
    if(TempRFState & RF_STOP){
        pRFFrame = buffer;
        RFFramePos = 0;
        RFFrameSize = 2;
        
        WriteRegister(REG_DIOMAPPING1, (RegistersCfg[REG_DIOMAPPING1] & 0x3F) | RF_DIOMAPPING1_DIO0_01); // DIO0 is "PAYLOADREADY"
    	WriteRegister(REG_SYNCCONFIG, (RegistersCfg[REG_SYNCCONFIG] & 0xBF) | RF_SYNC_FIFOFILL_AUTO);
                
        //RegIrqEnLow |= 0x04; // Enables Port A pin 2 interrupt DIO0 (PAYLOADREADY)

        SetRFMode(RF_RECEIVER);
        EnableTimeOut(TRUE);
        RFState |= RF_BUSY;
        RFState &= ~RF_STOP;
        RFState &= ~RF_TIMEOUT;
        return;
    }
    else if(TempRFState & RF_RX_DONE){
       
        SetRFMode(RF_SLEEP);    
        RFFrameSize = ReceiveByte();
        for(ByteCounter = 0, RFFramePos = 0; ByteCounter < RFFrameSize; ){
        		pRFFrame[RFFramePos++] = ReceiveByte();
            ByteCounter++; 
        }
        
        *size = RFFrameSize;
        *pReturnCode = OK;
        RFState |= RF_STOP;
        EnableTimeOut(FALSE);
        RFState &= ~RF_RX_DONE;
        //RegIrqEnLow &= ~0x04; // Disables Port A pin 2 interrupt DIO0 (PAYLOADREADY)
        //SPINss(1);
        return;
    }
    else if(TempRFState & RF_ERROR){
        
        SetRFMode(RF_SLEEP);
        
        RFState |= RF_STOP;
        RFState &= ~RF_ERROR;
        *pReturnCode = ERROR;
        //RegIrqEnLow &= ~0x04; // Disables Port A pin 2 interrupt DIO0 (PAYLOADREADY)
        //SPINss(1);
        return;
    }
    else if(TempRFState & RF_TIMEOUT){
        
        SetRFMode(RF_SLEEP);
        
        RFState |= RF_STOP;
        RFState &= ~RF_TIMEOUT;
        EnableTimeOut(FALSE);
        *pReturnCode = RX_TIMEOUT;
        //RegIrqEnLow &= ~0x04; // Disables Port A pin 2 interrupt DIO0 (PAYLOADREADY)
        //SPINss(1);
        return;
    }
} // void ReceiveRfFrame(uint8_t *buffer, uint8_t size, uint8_t *pReturnCode)

/*******************************************************************
** SendByte : Sends a data to the transceiver through the SPI     **
**            interface                                           **
********************************************************************
** In  : b                                                        **
** Out : -                                                        **
*******************************************************************/
void SendByte(uint8_t b){
	WriteRegister(REG_FIFO, b); // SPI burst mode not used in this implementation
} // void SendByte(uint8_t b)

/*******************************************************************
** ReceiveByte : Receives a data from the transceiver through the **
**               SPI interface                                    **                                                
********************************************************************
** In  : -                                                        **
** Out : b                                                        **
*******************************************************************/
uint8_t ReceiveByte(void){
	return ReadRegister(REG_FIFO); //SPI burst mode not used in this implementation
}
/*******************************************************************
** Transceiver specific functions                                 **
*******************************************************************/

/*******************************************************************
** ReadRssi : Reads the Rssi value from the SX1231                **
********************************************************************
** In  : -                                                        **
** Out : value                                                    **
*******************************************************************/
uint16_t ReadRssi(void){ // Must be called while in RX
	uint16_t value;
	WriteRegister(REG_RSSICONFIG, RegistersCfg[REG_RSSICONFIG] | RF_RSSI_START); // Triggers RSSI measurement
	while ((ReadRegister(REG_RSSICONFIG) & RF_RSSI_DONE) == 0x00);               // Waits for RSSI measurement to be completed
	value = ReadRegister(REG_RSSIVALUE);                                         // Reads the RSSI result
	return value;
}

/*******************************************************************
** ReadFei : Triggers FEI measurement and returns its value       **
********************************************************************
** In  : -                                                        **
** Out : value                                                    **
*******************************************************************/
int16_t ReadFei(void){ // Must be called while in RX
	int16_t value;
	WriteRegister(REG_AFCFEI, RegistersCfg[REG_AFCFEI] | RF_AFCFEI_FEI_START);   // Triggers FEI measurement
	while ((ReadRegister(REG_AFCFEI) & RF_AFCFEI_FEI_DONE) == 0x00);             // Waits for FEI measurement to be completed
	value = ((ReadRegister(REG_FEIMSB) << 8) | ReadRegister(REG_FEILSB));        // Reads the FEI result
	return value;
}

/*******************************************************************
** AutoFreqControl : Calibrates the receiver frequency to the     **
**               transmitter frequency                            **
********************************************************************
** In  : -                                                        **
** Out : *pReturnCode                                             **
*******************************************************************/
void AutoFreqControl(uint8_t *pReturnCode){ // Must be called while in RX
	WriteRegister(REG_AFCFEI, RegistersCfg[REG_AFCFEI] | RF_AFCFEI_AFC_START);   // Triggers AFC measurement
	while ((ReadRegister(REG_AFCFEI) & RF_AFCFEI_AFC_DONE) == 0x00);             // Waits for AFC measurement to be completed
   *pReturnCode = OK;
}

/*******************************************************************
** Utility functions                                              **
*******************************************************************/

#if 0
/*******************************************************************
** Wait : This routine uses the counter A&B to create a delay     **
**        using the RC ck source                                  **
********************************************************************
** In   : cntVal                                                  **
** Out  : -                                                       **
*******************************************************************/
void Wait(uint16_t cntVal){
    RegCntOn &= 0xFC;                              // Disables counter A&B
    RegEvnEn &= 0x7F;                              // Disables events from the counter A&B
    RegEvn = 0x80;                                 // Clears the event from the CntA on the event register
    RegCntCtrlCk =  (RegCntCtrlCk & 0xFC) | 0x01;  // Selects RC frequency as clock source for counter A&B
    RegCntConfig1 |= 0x34;                         // A&B counters count up, counter A&B are in cascade mode
    RegCntA       = (uint8_t)(cntVal);                 // LSB of cntVal
    RegCntB       = (uint8_t)(cntVal >> 8);            // MSB of cntVal
    RegEvnEn      |= 0x80;                         // Enables events from CntA
    RegEvn        |= 0x80;                         // Clears the event from the CntA on the event register
    asm("clrb %stat, #0");                         // Clears the event on the CoolRISC status register
    RegCntOn      |= 0x03;                         // Enables counter A&B
    do{
        asm("halt");
    }while ((RegEvn & 0x80) == 0x00);              // Waits the event from counter A
    RegCntOn      &= 0xFE;                         // Disables counter A
    RegEvnEn      &= 0x7F;                         // Disables events from the counter A
    RegEvn        |= 0x80;                         // Clears the event from the CntA on the event register
    asm("clrb %stat, #0");                         // Clears the event on the CoolRISC status register
} // void Wait(uint16_t cntVal)
#endif

/*******************************************************************
** EnableTimeOut : Enables/Disables the software RF frame timeout **
********************************************************************
** In  : enable                                                   **
** Out : -                                                        **
*******************************************************************/
void EnableTimeOut(uint8_t enable){
	TIM1_TimeBaseInit(15, TIM1_COUNTERMODE_UP, RFFrameTimeOut, 0);
	if(enable)
	{
		TIM1_Cmd(ENABLE);
	}
	else
	{
		TIM1_Cmd(DISABLE);
	}
} // void EnableTimeOut(uint8_t enable)

#if 0
/*******************************************************************
** InvertByte : Inverts a byte. MSB -> LSB, LSB -> MSB            **
********************************************************************
** In  : b                                                        **
** Out : b                                                        **
*******************************************************************/
uint8_t InvertByte(uint8_t b){
    asm("   move %r0, #0x08");
    asm("LoopInvertByte:");
    asm("   shl  %r3");
    asm("   shrc %r2");
    asm("   dec  %r0");
    asm("   jzc  LoopInvertByte");
} // uint8_t InvertByte(uint8_t b)
#endif

/*******************************************************************
** Handle_Irq_Pa1 : Handles the interruption from the Pin 1 of    **
**                  Port A                                        **
********************************************************************
** In  : -                                                        **
** Out : -                                                        **
*******************************************************************/
void Handle_Irq_Pa1 (void){ // DIO1 (Not used in this implementation) 
        
} //End Handle_Irq_Pa1

/*******************************************************************
** Handle_Irq_Pa2 : Handles the interruption from the Pin 2 of    **
**                  Port A                                        **
********************************************************************
** In  : -                                                        **
** Out : -                                                        **
*******************************************************************/
void Handle_Irq_Pa2 (void){ // DIO0 = PAYLOADREADY in RX
     
     RFState |= RF_RX_DONE;
     RFState &= ~RF_BUSY;
     
} //End Handle_Irq_Pa2

/*******************************************************************
** Handle_Irq_CntA : Handles the interruption from the Counter A  **
********************************************************************
** In              : -                                            **
** Out             : -                                            **
*******************************************************************/
void Handle_Irq_CntA (void){
    RFState |= RF_TIMEOUT;
    RFState &= ~RF_BUSY;
} //End Handle_Irq_CntA


