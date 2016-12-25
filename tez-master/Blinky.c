/*----------------------------------------------------------------------------
 *      
 *----------------------------------------------------------------------------
 *      Name:    BLINKY.C
 *      Purpose: Bare Metal example program
 *----------------------------------------------------------------------------
 *      
 *      Copyright (c) 2006-2014 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "MKL25Z4.h"                    // Device header
#include "TIspi.h"											//spi.h
#include <math.h>


#define 	WRITE_BURST     		0x40						//write burst
#define 	READ_SINGLE     		0x80						//read single
#define 	READ_BURST      		0xC0						//read burst
#define 	BYTES_IN_RXFIFO     0x7F  					//byte number in RXfifo

#define RADIO_BURST_ACCESS   0x40
#define RADIO_SINGLE_ACCESS  0x00
#define RADIO_READ_ACCESS    0x80
#define RADIO_WRITE_ACCESS   0x00

/* Bit fields in the chip status byte */
#define STATUS_CHIP_RDYn_BM             0x80
#define STATUS_STATE_BM                 0x70
#define STATUS_FIFO_BYTES_AVAILABLE_BM  0x0F
//CC1120 ISR Action definitions
#define ISR_ACTION_REQUIRED     1
#define ISR_IDLE                0
#define RX_FIFO_ERROR       0x11


/**********************************************************************
***********							 Macros       		  	*************************
***********************************************************************	
	Aim  : Macros are defined here
	NOTE : 
**********************************************************************/

							/*LEDS*/
#define RED_OFF 		(FPTB->PSOR |= (1<<18))
#define RED_ON  		(FPTB->PCOR |= (1<<18),FPTB->PSOR |= (1<<19),(FPTD->PSOR  = 0x01))
#define GREEN_OFF 	(FPTB->PSOR |= (1<<19))
#define GREEN_ON  	(FPTB->PSOR |= (1<<18),FPTB->PCOR |= (1<<19),(FPTD->PSOR  = 0x01))
#define YELLOW_OFF 	(FPTB->PSOR |= (1<<18),FPTB->PSOR |= (1<<19),(FPTD->PSOR  = 0x01))
#define YELLOW_ON  	(FPTB->PCOR |= (1<<18),FPTB->PCOR |= (1<<19),(FPTD->PSOR  = 0x01))

//#define SpiStart()			FPTD->PCOR |= (1UL<<4);                             	// CS=low,  SPI start
//#define SpiStop()				FPTD->PSOR |= (1UL<<4);                             	// CS=high, SPI stop

// test
#define SpiStart()			FPTD->PCOR |= (1UL<<3);                             	// CS=low,  SPI start
#define SpiStop()				FPTD->PSOR |= (1UL<<3);                             	// CS=high, SPI stop

//#define SpiStart()			PTA->PCOR |= (1UL);                             	// CS=low,  SPI start
//#define SpiStop()				PTA->PSOR |= (1UL);                             	// CS=high, SPI stop

#define wait_CHIP_RDYn	while((FPTD->PDIR & 0x80)!= 0)											//Wait for CHIP_RDYn signal
//#define wait_CHIP_RDYn	while((SPI_Send(0x00) & 0x80) != 0);

/**********************************************************************
***********							 Global Values		  	*************************
***********************************************************************	
	Aim  : Global registers are defined here
	NOTE : 
**********************************************************************/
char test[22];
char C[4];
uint32_t hede;
char config;
char toto[22]="AMK POKUMANI";
char got[22];
char packetSemaphore;
int  packetCounter = 0;
/**********************************************************************
***********							 Systick					  	*************************
***********************************************************************	
	Aim  : initialize Systick Timer
	NOTE : 1 ms Ticks
**********************************************************************/
volatile uint32_t msTicks;                            /* counts 1ms timeTicks */
void SysTick_Handler(void) {
  msTicks++;                        /* increment counter necessary in Delay() */
}
__INLINE static void Delay (uint32_t dlyTicks) {
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks);
}
/**********************************************************************
***********							 LED_Init							*************************
***********************************************************************	
	Aim  : initialize the leds
	NOTE : 18>Red---19>Green---- 18+19>Yellow
**********************************************************************/
void LED_Init(void) {

   
  PORTB->PCR[18] = PORT_PCR_MUX(1);                     /* Pin PTB18 is GPIO-RED */ 
  PORTB->PCR[19] = PORT_PCR_MUX(1);                     /* Pin PTB19 is GPIO-GREEN */
	PORTD->PCR[1]  = PORT_PCR_MUX(1);                     /* Pin PTD1 is GPIO-BLUE */
	FPTB->PDDR     |= (1UL<<18) | (1UL<<19);						 	/* enable PTB18/19 as Output */
	FPTD->PDDR     |= (1UL);															// enable PTD1 as output
	//Turn off leds
	YELLOW_OFF;	GREEN_OFF;	RED_OFF;
	// Turn on leds 1 by 1 
	YELLOW_ON; Delay(1000);	GREEN_ON; Delay(1000);	RED_ON; Delay(1000);
	//Turn off leds
	YELLOW_OFF;	GREEN_OFF;	RED_OFF;
	
}
/**********************************************************************
***********							 SPI_Init							*************************
***********************************************************************	
	Aim  : initialize SPI for Kl25
	NOTE : 
**********************************************************************/
	void spi_init(void){
		  	 																								
		int dly;
		
    SIM->SCGC4 |= SIM_SCGC4_SPI1_MASK;        	 								//Enable SPI1 clock  
		
		
		//manual ss pin 
//		PORTD->PCR[4] |= PORT_PCR_MUX(1); 													//SS pin gpio
//		PTD->PDDR     |= (1UL<<4);																	//SS pin output PD4
		
		PORTD->PCR[3] |= PORT_PCR_MUX(1); 													//SS pin gpio
		PTD->PDDR     |= (1UL<<3);																	//SS pin output PD3
		
//		PORTA->PCR[1]   |= PORT_PCR_MUX(1); 
//		PTA->PDDR   	  |= (1UL);																	
		
		PORTD->PCR[2] |= PORT_PCR_MUX(1); 													//TI_Reset pin gpio
		PTD->PDDR     |= (1UL<<2);																	//TI_Reset pin output PD2
		
		//PORTD->PCR[0] = PORT_PCR_MUX(0x2);           							//Set PTD4 to mux 2   (SS)
		PORTD->PCR[5] = PORT_PCR_MUX(0x02);           							//Set PTD5 to mux 2   (clk)
		PORTD->PCR[6] = PORT_PCR_MUX(0x02);           							//Set PTD6 to mux 2   (Mosi)
		PORTD->PCR[7] = PORT_PCR_MUX(0x02);           							//Set PTD7 to mux 2   (Miso)
		
		SPI1->C1 = SPI_C1_MSTR_MASK;         											//Set SPI0 to Master   
		SPI1->C2 = SPI_C2_MODFEN_MASK;                           	//Master SS pin acts as slave select output        
		//SPI1->BR = (SPI_BR_SPPR(0x111) | SPI_BR_SPR(0x0100));     //Set baud rate prescale divisor to 3 & set baud rate divisor to 32 for baud rate of 15625 hz        
//		SPI1->BR |= 0x30;
			
			
			SPI1->BR |= 0x43;
//			SPI1->BR |= 0x45; // test
//		SPI1->C1 |=  (1UL << 3) ; 										//SPI MOD 3
//		SPI1->C1 |=  (1UL << 2) ; 
		
		dly=1000;
		while(dly--);
		SPI1->C1 |= 0x40;																					//SPI1 Enable
		PTD->PSOR |= (1UL<<4);
		PTD->PSOR |= (1UL<<2);
		

	}
	// TEST macros
#define TRXEM_SPI_TX(x) 	SPI1->D = x
#define TRXEM_SPI_RX()		SPI1->D
#define TRXEM_SPI_WAIT_DONE()		(while(!(SPI_S_SPTEF_MASK & SPI1->S) || (SPI_S_SPRF_MASK & SPI1->S) );)
/**********************************************************************
***********							 SPI_SEND							*************************
***********************************************************************	
	Aim  :  send and recieve data via SPI
	NOTE : 
**********************************************************************/
char SPI_Send(char Data)
{
	while(!(SPI_S_SPTEF_MASK & SPI1->S));   
	SPI1->D = Data;																								//Write Data
	while(!(SPI_S_SPRF_MASK & SPI1->S)); 
	return 	SPI1->D;																							//Read Data

}
char testt;
char x;
/**********************************************************************
***********							 TI_HW_Reset							*************************
***********************************************************************	
	Aim  : Hardware Reset using MCU
	NOTE :active low for 200 ns, other times high
**********************************************************************/
void TI_HW_Reset(void)
{
	PTD->PCOR |= (1UL<<2);
	Delay(0x05);
	PTD->PSOR |= (1UL<<2);
}

/**********************************************************************
***********							 TI_Init							*************************
***********************************************************************	
	Aim  : wait for initialization of CC1120
	NOTE :
**********************************************************************/
void TI_Init(void)
{
	TI_HW_Reset();
	
	SpiStart();																										//Start SPI by CSn Low
	wait_CHIP_RDYn; 																							//Wait for TI's christal to be stabilized	
//	while((SPI_Send(0x00) & 0x80) != 0);
//	x= SPI_Send(0x00);
	//	Delay(0x5000);
	SpiStop();																										//Stop SPI by CSn High
	
}

/**********************************************************************
***********							 TI_Write							*************************
***********************************************************************	
	Aim  : write 1 byte data to chip via SPI
	NOTE :[ R/W bit (0)] + [ Burst bit (0)] + [6 bit addres] + 8 bit data
**********************************************************************/
void TI_WriteByte( char addr,  char data)
{
//int dly;
//  SpiStart();																										//Start SPI by CSn Low
//	wait_CHIP_RDYn; 																							//Wait for TI's christal to be stabilized
	SPI_Send(addr);																							  //Send 1 byte addr and write command
	SPI_Send(data);																								//Send 1 byte data 
//	SpiStop();	                               										//Stop SPI by CSn High
}

/**********************************************************************
***********							 TI_Read							*************************
***********************************************************************	
	Aim  : Read 1 byte data from chip via SPI
	NOTE :[ R/W bit (1)] + [ Burst bit (0)] + [6 bit addres]
**********************************************************************/
char TI_ReadByte(char addr)
{
  char data = 0;
	
//	SpiStart();																										//Start SPI by CSn Low
//	wait_CHIP_RDYn; 																							//Wait for TI's christal to be stabilized
	//SPI_Send( READ_SINGLE | addr);																// R/w bit (1) + Burst bit (0)+ 6 bit addres
	data = SPI_Send(0x00);             														// Data read (read 1byte data) via dummy write
//	SpiStop();																										//Stop SPI by CSn High
  return data;    
}
/**********************************************************************
***********							 TI_Write_brst							*******************
***********************************************************************	
	Aim  : burst write to TI chip
	NOTE :[ R/W bit (0)] + [ Burst bit (1)] + [6 bit addres stars with 0x2F00]
**********************************************************************/
int TI_Write_brst(int addr,char* buf,int len)
{
  int i = 0;

//	SpiStart();					                             	  	//Start SPI by CSn Low
//	wait_CHIP_RDYn; 																			//Wait for TI's christal to be stabilized
	//SPI_Send ( WRITE_BURST | addr );											// Send the adrr
		
	for(i = 0; i < len; i++)              								// Burst Write the data 
		{
			SPI_Send (buf[i]);
		}
	 
//	SpiStop();																						//Stop SPI by CSn High
	return len;  
}
/**********************************************************************
***********							 TI_READ_brst							*********************
***********************************************************************	
	Aim  : burst read to TI chip
	NOTE :[ R/W bit (1)] + [ Burst bit (1)] + [6 bit addres]
**********************************************************************/
int TI_Read_brst(int addr, char* buf,int len)
{	
	int i = 0;

//	SpiStart();					                             	  	//Start SPI by CSn Low
//	wait_CHIP_RDYn; 																			//Wait for TI's christal to be stabilized
	//SPI_Send ( READ_BURST | addr );			 test									// Address byte 1
	for(i = 0; i < len; i++)                    				  // Write data in loop
		{
			buf[i] = SPI_Send(0x00);													//write data to buffer with size of "len"
		}
//	SpiStop();																						//Stop SPI by CSn High
	return len;
}

/**********************************************************************
***********							 Write TI_Command							*****************
***********************************************************************	
	Aim  : command strobe acces
	NOTE :[ R/W bit (0)] + [ Burst bit (0)] + [6 bit addres]  
				 No data is expected. Chip_status_Byte is returned from chip
**********************************************************************/
void TI_Command( char command )
{
	SpiStart();																								//Start SPI by CSn Low
	wait_CHIP_RDYn; 																					//Wait for TI's christal to be stabilized
	SPI_Send(command);																				//Send chip command
	SpiStop();	                               								//Stop SPI by CSn High
}

char TI_Command_Read(char command)
{
	char ret;
	SpiStart();																								//Start SPI by CSn Low
	wait_CHIP_RDYn; 																					//Wait for TI's christal to be stabilized
	SPI_Send(command|READ_SINGLE);														//Send chip command with READ bit
	ret=SPI_Send(0x00);																				//send dummy byte so read status byte
	SpiStop();	                               								//Stop SPI by CSn High
	return ret;
}
/**********************************************************************
***********							DelayUs						*********************
***********************************************************************	
	Aim  : Blind Delay
	NOTE : microsecond delay (To be Calibrated)
**********************************************************************/
char waitUs(long t)																
{
do
	{
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
//	__NOP();
//	__NOP();
//	__NOP();
	} while(--t);
	return 1;
}
/**********************************************************************
***********							 trxReadWriteBurstSingle							*******************
***********************************************************************	
	Aim  : TEST issue 
	NOTE :
**********************************************************************/
static void trxReadWriteBurstSingle(uint8_t addr,char *pData,uint16_t len)
{
	
	/* Communicate len number of bytes: if RX - the procedure sends 0x00 to push bytes from slave*/
  if(addr&RADIO_READ_ACCESS)
  {
    if(addr&RADIO_BURST_ACCESS)
    {
     TI_Read_brst(addr,pData,len);
    }
    else
    {
     TI_ReadByte(addr);
    }
  }
  else
  {
    if(addr&RADIO_BURST_ACCESS)
    {
      /* Communicate len number of bytes: if TX - the procedure doesn't overwrite pData */
      TI_Write_brst(addr,pData,len);
    }
    else
    {
      TI_WriteByte(addr,*pData);
    }
  }
  return;
}


/**********************************************************************
***********							 trx16BitRegAccess							*******************
***********************************************************************	
	Aim  : TEST issue 
	NOTE :
**********************************************************************/
char trx16BitRegAccess(uint8_t accessType, uint8_t extAddr, uint8_t regAddr, char *pData, uint8_t len)
{
  uint8_t readValue;

  SpiStart();					                             	  	//Start SPI by CSn Low
  wait_CHIP_RDYn; 
  /* send extended address byte with access type bits set */
  readValue =SPI_Send(accessType|extAddr); //Storing chip status */
  SPI_Send(regAddr);
  /* Communicate len number of bytes */
  trxReadWriteBurstSingle(accessType|extAddr,pData,len);
  SpiStop();
  /* return the status byte value */
  return(readValue);
}

/**********************************************************************
***********							 trx8BitRegAccess							*******************
***********************************************************************	
	Aim  : TEST issue 
	NOTE :
**********************************************************************/
uint8_t trx8BitRegAccess(uint8_t accessType, uint8_t addrByte, char *pData, uint16_t len)
{
  uint8_t readValue;

  /* Pull CS_N low and wait for SO to go low before communication starts */
  SpiStart();
  wait_CHIP_RDYn;
  /* send register address byte */
  readValue = SPI_Send(accessType|addrByte);  /* Storing chip status */
//  while(!(SPI_S_SPTEF_MASK & SPI1->S));   
//	SPI1->D = (accessType|addrByte);																								//Write Data
//	while(!(SPI_S_SPRF_MASK & SPI1->S)); 
//	readValue = SPI1->D;
	trxReadWriteBurstSingle(accessType|addrByte,pData,len);
  SpiStop();
  /* return the status byte value */
  return(readValue);
}

/******************************************************************************
 * @fn          cc112xSpiReadReg
 *
 * @brief       Read value(s) from config/status/extended radio register(s).
 *              If len  = 1: Reads a single register
 *              if len != 1: Reads len register values in burst mode 
 *
 * input parameters
 *
 * @param       addr   - address of first register to read
 * @param       *pData - pointer to data array where read bytes are saved
 * @param       len   - number of bytes to read
 *
 * output parameters
 *
 * @return      rfStatus_t
 */
uint8_t cc112xSpiReadReg(uint16_t addr, char *pData, uint8_t len)
{
  uint8_t tempExt  = (uint8_t)(addr>>8);
  uint8_t tempAddr = (uint8_t)(addr & 0x00FF);
  uint8_t rc;
  
  /* Checking if this is a FIFO access -> returns chip not ready  */
  if((CC112X_SINGLE_TXFIFO<=tempAddr)&&(tempExt==0)) return STATUS_CHIP_RDYn_BM;
  
  /* Decide what register space is accessed */
  if(!tempExt)
  {
    rc = trx8BitRegAccess((RADIO_BURST_ACCESS|RADIO_READ_ACCESS),tempAddr,pData,len);
  }
  else if (tempExt == 0x2F)
  {
    rc = trx16BitRegAccess((RADIO_BURST_ACCESS|RADIO_READ_ACCESS),tempExt,tempAddr,pData,len);
  }
  return (rc);
}

/*******************************************************************************
 * @fn          trxSpiCmdStrobe
 *
 * @brief       Send command strobe to the radio. Returns status byte read
 *              during transfer of command strobe. Validation of provided
 *              is not done. Function assumes chip is ready.
 *
 * input parameters
 *
 * @param       cmd - command strobe
 *
 * output parameters
 *
 * @return      status byte
 */
uint8_t trxSpiCmdStrobe(uint8_t cmd)
{
    uint8_t rc;
    SpiStart();
		wait_CHIP_RDYn;
    rc = SPI_Send(cmd);
    SpiStop();
    return(rc);
}

/******************************************************************************
 * @fn          cc112xSpiWriteReg
 *
 * @brief       Write value(s) to config/status/extended radio register(s).
 *              If len  = 1: Writes a single register
 *              if len  > 1: Writes len register values in burst mode 
 *
 * input parameters
 *
 * @param       addr   - address of first register to write
 * @param       *pData - pointer to data array that holds bytes to be written
 * @param       len    - number of bytes to write
 *
 * output parameters
 *
 * @return      rfStatus_t
 */
uint8_t cc112xSpiWriteReg(uint16_t addr, char *pData, uint8_t len)
{
  uint8_t tempExt  = (uint8_t)(addr>>8);
  uint8_t tempAddr = (uint8_t)(addr & 0x00FF);
  uint8_t rc;
  
  /* Checking if this is a FIFO access -> returns chip not ready  */
  if((CC112X_SINGLE_TXFIFO<=tempAddr)&&(tempExt==0)) return STATUS_CHIP_RDYn_BM;
  
  /* Decide what register space is accessed */
  if(!tempExt)
  {
    rc = trx8BitRegAccess((RADIO_BURST_ACCESS|RADIO_WRITE_ACCESS),tempAddr,pData,len);
  }
  else if (tempExt == 0x2F)
  {
    rc = trx16BitRegAccess((RADIO_BURST_ACCESS|RADIO_WRITE_ACCESS),tempExt,tempAddr,pData,len);
  }
  return (rc);
}
/*******************************************************************************
 * @fn          cc112xSpiWriteTxFifo
 *
 * @brief       Write pData to radio transmit FIFO.
 *
 * input parameters
 *
 * @param       *pData - pointer to data array that is written to TX FIFO
 * @param       len    - Length of data array to be written
 *
 * output parameters
 *
 * @return      rfStatus_t (uint8_t)
 */
uint8_t cc112xSpiWriteTxFifo(char *pData, uint8_t len)
{
  uint8_t rc;
  rc = trx8BitRegAccess(0x00,CC112X_BURST_TXFIFO, pData, len);
  return (rc);
}
/*******************************************************************************
 * @fn          cc112xSpiReadRxFifo
 *
 * @brief       Reads RX FIFO values to pData array
 *
 * input parameters
 *
 * @param       *pData - pointer to data array where RX FIFO bytes are saved
 * @param       len    - number of bytes to read from the RX FIFO
 *
 * output parameters
 *
 * @return      rfStatus_t
 */
uint8_t cc112xSpiReadRxFifo(char * pData, uint8_t len)
{
  uint8_t rc;
  rc = trx8BitRegAccess(0x00,CC112X_BURST_RXFIFO, pData, len);
  return (rc);
}

/******************************************************************************
 * @fn      cc112xGetTxStatus(void)
 *          
 * @brief   This function transmits a No Operation Strobe (SNOP) to get the 
 *          status of the radio and the number of free bytes in the TX FIFO.
 *          
 *          Status byte:
 *          
 *          ---------------------------------------------------------------------------
 *          |          |            |                                                 |
 *          | CHIP_RDY | STATE[2:0] | FIFO_BYTES_AVAILABLE (free bytes in the TX FIFO |
 *          |          |            |                                                 |
 *          ---------------------------------------------------------------------------
 *
 *
 * input parameters
 *
 * @param   none
 *
 * output parameters
 *         
 * @return  rfStatus_t 
 *
 */
uint8_t cc112xGetTxStatus(void)
{
    return(trxSpiCmdStrobe(CC112X_SNOP));
}

/******************************************************************************
 *
 *  @fn       cc112xGetRxStatus(void)
 *
 *  @brief   
 *            This function transmits a No Operation Strobe (SNOP) with the 
 *            read bit set to get the status of the radio and the number of 
 *            available bytes in the RXFIFO.
 *            
 *            Status byte:
 *            
 *            --------------------------------------------------------------------------------
 *            |          |            |                                                      |
 *            | CHIP_RDY | STATE[2:0] | FIFO_BYTES_AVAILABLE (available bytes in the RX FIFO |
 *            |          |            |                                                      |
 *            --------------------------------------------------------------------------------
 *
 *
 * input parameters
 *
 * @param     none
 *
 * output parameters
 *         
 * @return    rfStatus_t 
 *
 */
uint8_t cc112xGetRxStatus(void)
{
    return(trxSpiCmdStrobe(CC112X_SNOP | RADIO_READ_ACCESS));
}

/*******************************************************************************
*   @fn         manualCalibration
*
*   @brief      Calibrates radio according to CC112x errata
*
*   @param      none
*
*   @return     none
*/
#define VCDAC_START_OFFSET 2
#define FS_VCO2_INDEX 0
#define FS_VCO4_INDEX 1
#define FS_CHP_INDEX 2

static void RX_manualCalibration(void) {

    char original_fs_cal2;
    char calResults_for_vcdac_start_high[3];
    char calResults_for_vcdac_start_mid[3];
    char marcstate;
    char writeByte;

    // 1) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
    writeByte = 0x00;
    cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);

    // 2) Start with high VCDAC (original VCDAC_START + 2):
    cc112xSpiReadReg(CC112X_FS_CAL2, &original_fs_cal2, 1);
    writeByte = original_fs_cal2 + VCDAC_START_OFFSET;
    cc112xSpiWriteReg(CC112X_FS_CAL2, &writeByte, 1);

    // 3) Calibrate and wait for calibration to be done
    //   (radio back in IDLE state)
    trxSpiCmdStrobe(CC112X_SCAL);

    do {
        cc112xSpiReadReg(CC112X_MARCSTATE, &marcstate, 1);
    } while (marcstate != 0x41);

    // 4) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained with
    //    high VCDAC_START value
    cc112xSpiReadReg(CC112X_FS_VCO2,
                     &calResults_for_vcdac_start_high[FS_VCO2_INDEX], 1);
    cc112xSpiReadReg(CC112X_FS_VCO4,
                     &calResults_for_vcdac_start_high[FS_VCO4_INDEX], 1);
    cc112xSpiReadReg(CC112X_FS_CHP,
                     &calResults_for_vcdac_start_high[FS_CHP_INDEX], 1);

    // 5) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
    writeByte = 0x00;
    cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);

    // 6) Continue with mid VCDAC (original VCDAC_START):
    writeByte = original_fs_cal2;
    cc112xSpiWriteReg(CC112X_FS_CAL2, &writeByte, 1);

    // 7) Calibrate and wait for calibration to be done
    //   (radio back in IDLE state)
    trxSpiCmdStrobe(CC112X_SCAL);

    do {
        cc112xSpiReadReg(CC112X_MARCSTATE, &marcstate, 1);
    } while (marcstate != 0x41);

    // 8) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained
    //    with mid VCDAC_START value
    cc112xSpiReadReg(CC112X_FS_VCO2,
                     &calResults_for_vcdac_start_mid[FS_VCO2_INDEX], 1);
    cc112xSpiReadReg(CC112X_FS_VCO4,
                     &calResults_for_vcdac_start_mid[FS_VCO4_INDEX], 1);
    cc112xSpiReadReg(CC112X_FS_CHP,
                     &calResults_for_vcdac_start_mid[FS_CHP_INDEX], 1);

    // 9) Write back highest FS_VCO2 and corresponding FS_VCO
    //    and FS_CHP result
    if (calResults_for_vcdac_start_high[FS_VCO2_INDEX] >
        calResults_for_vcdac_start_mid[FS_VCO2_INDEX]) {
        writeByte = calResults_for_vcdac_start_high[FS_VCO2_INDEX];
        cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);
        writeByte = calResults_for_vcdac_start_high[FS_VCO4_INDEX];
        cc112xSpiWriteReg(CC112X_FS_VCO4, &writeByte, 1);
        writeByte = calResults_for_vcdac_start_high[FS_CHP_INDEX];
        cc112xSpiWriteReg(CC112X_FS_CHP, &writeByte, 1);
    } else {
        writeByte = calResults_for_vcdac_start_mid[FS_VCO2_INDEX];
        cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);
        writeByte = calResults_for_vcdac_start_mid[FS_VCO4_INDEX];
        cc112xSpiWriteReg(CC112X_FS_VCO4, &writeByte, 1);
        writeByte = calResults_for_vcdac_start_mid[FS_CHP_INDEX];
        cc112xSpiWriteReg(CC112X_FS_CHP, &writeByte, 1);
    }
}

/*******************************************************************************
*   @fn         runRX
*
*   @brief      Puts radio in RX and waits for packets. Function assumes
*               that status bytes are appended in the RX_FIFO
*               Update packet counter and display for each packet received.
*		
*		@Note 			1. GPIO2 has to be set up with GPIO2_CFG = 0x06-->   PKT_SYNC_RXTX for correct interupt
//              2. Packet engine has to be set up with status bytes enabled PKT_CFG1.APPEND_STATUS = 1
*   @param      none
*
*   @return     none
*********************************************************************************/
static void runRX(void) 
{
	
	char rxBuffer[128] = {0};
  char rxBytes;
  char marcState;
	
	// Calibrate radio according to errata
  RX_manualCalibration();
	
	// Set radio in RX
  trxSpiCmdStrobe(CC112X_SRX);
	
	
	// Wait for packet received interrupt
	if(packetSemaphore == ISR_ACTION_REQUIRED) 
		{
			// Read number of bytes in RX FIFO
			cc112xSpiReadReg(CC112X_NUM_RXBYTES, &rxBytes, 1);

			// Check that we have bytes in FIFO
			if(rxBytes != 0) 
				{
					// Read MARCSTATE to check for RX FIFO error
					cc112xSpiReadReg(CC112X_MARCSTATE, &marcState, 1);

					// Mask out MARCSTATE bits and check if we have a RX FIFO error
					if((marcState & 0x1F) == RX_FIFO_ERROR) 
						{
						// Flush RX FIFO
						trxSpiCmdStrobe(CC112X_SFRX);
						} 
						else 
						{
							// Read n bytes from RX FIFO
							cc112xSpiReadRxFifo(rxBuffer, rxBytes);

							// Check CRC ok (CRC_OK: bit7 in second status byte)
							// This assumes status bytes are appended in RX_FIFO
							// (PKT_CFG1.APPEND_STATUS = 1)
							// If CRC is disabled the CRC_OK field will read 1
							if(rxBuffer[rxBytes - 1] & 0x80) 
								{
									// Update packet counter
									packetCounter++;
								}
						 }
				 }
				if(rxBuffer[0] != 0)
				{
					GREEN_ON;
				}
				// Reset packet semaphore
				packetSemaphore = ISR_IDLE;
				// Set radio back in RX
				trxSpiCmdStrobe(CC112X_SRX);
    }
}

#define PKTLEN	30  // 1 < PKTLEN < 126

#define GPIO3   0x04
#define GPIO2   0x08
#define GPIO0   0x80
/*******************************************************************************
*   @fn         createPacket
*
*   @brief      This function is called before a packet is transmitted. It fills
*               the txBuffer with a packet consisting of a length byte, two
*               bytes packet counter and n random bytes.
*
*               The packet format is as follows:
*               |--------------------------------------------------------------|
*               |           |           |           |         |       |        |
*               | pktLength | pktCount1 | pktCount0 | rndData |.......| rndData|
*               |           |           |           |         |       |        |
*               |--------------------------------------------------------------|
*                txBuffer[0] txBuffer[1] txBuffer[2]            txBuffer[PKTLEN]
*
*   @param       Pointer to start of txBuffer
*
*   @return      none
*/
static void createPacket(char txBuffer[]) 
{
	uint8_t i;
  txBuffer[0] = PKTLEN;                           // Length byte
  txBuffer[1] = (uint8_t) (packetCounter >> 8);     // MSB of packetCounter
  txBuffer[2] = (uint8_t)  packetCounter;           // LSB of packetCounter

    // Fill rest of buffer with random bytes
  for(i = 3; i < (PKTLEN + 1); i++) 
	{
		txBuffer[i] = 0xAB;
  }
}
/*******************************************************************************
*   @fn         manualCalibration
*
*   @brief      Calibrates radio according to CC112x errata
*
*   @param      none
*
*   @return     none
*/
#define VCDAC_START_OFFSET 2
#define FS_VCO2_INDEX 0
#define FS_VCO4_INDEX 1
#define FS_CHP_INDEX 2
static void TX_manualCalibration(void) 
{
  char original_fs_cal2;
  char calResults_for_vcdac_start_high[3];
  char calResults_for_vcdac_start_mid[3];
  char marcstate;
  char writeByte;

  // 1) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
  writeByte = 0x00;
  cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);

    // 2) Start with high VCDAC (original VCDAC_START + 2):
    cc112xSpiReadReg(CC112X_FS_CAL2, &original_fs_cal2, 1);
    writeByte = original_fs_cal2 + VCDAC_START_OFFSET;
    cc112xSpiWriteReg(CC112X_FS_CAL2, &writeByte, 1);

    // 3) Calibrate and wait for calibration to be done
    //   (radio back in IDLE state)
    trxSpiCmdStrobe(CC112X_SCAL);

    do {
        cc112xSpiReadReg(CC112X_MARCSTATE, &marcstate, 1);
    } while (marcstate != 0x41);

    // 4) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained with 
    //    high VCDAC_START value
    cc112xSpiReadReg(CC112X_FS_VCO2,
                     &calResults_for_vcdac_start_high[FS_VCO2_INDEX], 1);
    cc112xSpiReadReg(CC112X_FS_VCO4,
                     &calResults_for_vcdac_start_high[FS_VCO4_INDEX], 1);
    cc112xSpiReadReg(CC112X_FS_CHP,
                     &calResults_for_vcdac_start_high[FS_CHP_INDEX], 1);

    // 5) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
    writeByte = 0x00;
    cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);

    // 6) Continue with mid VCDAC (original VCDAC_START):
    writeByte = original_fs_cal2;
    cc112xSpiWriteReg(CC112X_FS_CAL2, &writeByte, 1);

    // 7) Calibrate and wait for calibration to be done
    //   (radio back in IDLE state)
    trxSpiCmdStrobe(CC112X_SCAL);

    do {
        cc112xSpiReadReg(CC112X_MARCSTATE, &marcstate, 1);
    } while (marcstate != 0x41);

    // 8) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained 
    //    with mid VCDAC_START value
    cc112xSpiReadReg(CC112X_FS_VCO2, 
                     &calResults_for_vcdac_start_mid[FS_VCO2_INDEX], 1);
    cc112xSpiReadReg(CC112X_FS_VCO4,
                     &calResults_for_vcdac_start_mid[FS_VCO4_INDEX], 1);
    cc112xSpiReadReg(CC112X_FS_CHP,
                     &calResults_for_vcdac_start_mid[FS_CHP_INDEX], 1);

    // 9) Write back highest FS_VCO2 and corresponding FS_VCO
    //    and FS_CHP result
    if (calResults_for_vcdac_start_high[FS_VCO2_INDEX] >
        calResults_for_vcdac_start_mid[FS_VCO2_INDEX]) {
        writeByte = calResults_for_vcdac_start_high[FS_VCO2_INDEX];
        cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);
        writeByte = calResults_for_vcdac_start_high[FS_VCO4_INDEX];
        cc112xSpiWriteReg(CC112X_FS_VCO4, &writeByte, 1);
        writeByte = calResults_for_vcdac_start_high[FS_CHP_INDEX];
        cc112xSpiWriteReg(CC112X_FS_CHP, &writeByte, 1);
    } else {
        writeByte = calResults_for_vcdac_start_mid[FS_VCO2_INDEX];
        cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);
        writeByte = calResults_for_vcdac_start_mid[FS_VCO4_INDEX];
        cc112xSpiWriteReg(CC112X_FS_VCO4, &writeByte, 1);
        writeByte = calResults_for_vcdac_start_mid[FS_CHP_INDEX];
        cc112xSpiWriteReg(CC112X_FS_CHP, &writeByte, 1);
    }
}
/*******************************************************************************
*   @fn         runTX
*
*   @brief      Continuously sends packets on button push until button is pushed
*               again. After the radio has gone into TX the function waits for
*               interrupt that packet has been sent. Updates packet counter and
*               display for each packet sent.
*
*   @param      none
*
*   @return    none
*/
static void runTX(void) 
{
	// Initialize packet buffer of size PKTLEN + 1
 // char txBuffer[PKTLEN+1] = {0}; "AMK POKUMANI";
	char txBuffer[PKTLEN+1] = "AMK POKUMANI";
	
	// Calibrate radio according to errata
   TX_manualCalibration();

	// Update packet counter
  packetCounter++;

  // Create a random packet with PKTLEN + 2 byte packet
  // counter + n x random bytes
  createPacket(txBuffer);

  // Write packet to TX FIFO
  cc112xSpiWriteTxFifo(txBuffer, sizeof(txBuffer));

  // Strobe TX to send packet
  trxSpiCmdStrobe(CC112X_STX);

  // Wait for interrupt that packet has been sent.
  // (Assumes the GPIO connected to the radioRxTxISR function is
  // set to GPIOx_CFG = 0x06)
  while(packetSemaphore != ISR_ACTION_REQUIRED);
  // Clear semaphore flag
  packetSemaphore = ISR_IDLE;
 
}
/*****************************************************************************************************/
//									Initialize Interrupt for CC1120's GPIO-2 and if necessary GPIO-3

/* -->  		PortD(4) connected to GPIO-2 of CC1120 for interrupt  */

/*****************************************************************************************************/
void IRQ_Init(void)
{
	
//	PORTD->PCR[4] |=PORT_PCR_MUX(1);							//Port D-4 is GPIO
	
	
	PORTA->PCR[12] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1) | PORT_PCR_PS(1) | PORT_PCR_IRQC(0x0A); //PTD4 as GPIO, Pull up, interrupt on falling edge
	PTA->PDDR     &= (0UL<<12);							  		//Port D-4 Input
	
//	PORTD->PCR[4] |= (1UL<< 19);		//IRQC = 1010
//	PORTD->PCR[4] |= (1UL<< 17);		//IRQC= 1010   Interrupt @ falling edge
//	PORTD->PCR[4] |= (1UL<< 8);		  //PCR mux 1 (GPIO)
	
	//Enable UART interrupt				
	//NVIC_EnableIRQ(UART0_IRQn);
	//NVIC_SetPriority(UART0_IRQn,1);
	
	//Enable PortD Hardware interrupt
	
	NVIC_EnableIRQ(PORTA_IRQn );							//Port D IRQ enable	
	NVIC_SetPriority(PORTA_IRQn ,2);					//Port D IRQ priority 2
//				NVIC_SetPriority(SysTick_IRQn ,2);				//SysTick Timer priority 
	
//					PORTD->PCR[4]= PORT_PCR_ISF_MASK | PORT_PCR_IRQC(10) ;
////				PORTD->PCR[4]= PORT_PCR_IRQC(10) ;
			
}

char asde;
/*****************************************************************************************************/
/*													Port D IRQ routine              																			   */
/*****************************************************************************************************/		
void PORTA_IRQHandler(void)
{
	/*
		DO IRQ JOB here... Than do not forget to Clear the interrupt!!!!!!!!
	*/
	
	packetSemaphore = ISR_ACTION_REQUIRED ;
	
	PORTA->PCR[12] |= PORT_PCR_ISF_MASK; 		// Clear ISF flag for clearing interrupt		
	NVIC_ClearPendingIRQ(PORTA_IRQn);			// clear pending int from KL25
}

/**********************************************************************
***********							Read Temp Sensor						*********************
***********************************************************************	
	Aim  : temp sensor digital readout
	NOTE : see DN-403 Application note!!!
**********************************************************************/
static int8_t tempRead(void) {
//Variables
char RegValue = 0;
char marcStatus;
char writeByte;
int ADCValue_I = 0;
char celsius = 0;
//String to put radio in debug mode
char txBuffer[18] =
{0x0F,0x28,0x02,0x90,0x42,0x1B,0x7E,0x1F,0xFE,0xCD,0x06,0x1B,0x0E,0xA1,0x0E,0xA4,0x00,0x3F};
//Constants for temperature calculation
float a = -3.3;
float b = 1000;
float c =-2629.9;
//Register settings specific for temp readout.
writeByte = 0x40;
cc112xSpiWriteReg( CC112X_DCFILT_CFG, &writeByte, 1); //Tempsens settings, bit 6 high
writeByte = 0x47;
cc112xSpiWriteReg( CC112X_MDMCFG1, &writeByte, 1); //Tempsens settings, single ADC, I channel
writeByte = 0x81;
cc112xSpiWriteReg( CC112X_CHAN_BW, &writeByte, 1); //Tempsens settings, bit 7 high Bypass ch filt.
writeByte = 0x00;
cc112xSpiWriteReg( CC112X_FREQ_IF_CFG, &writeByte, 1); //Tempsens settings, 0-IF
writeByte = 0x2A;
cc112xSpiWriteReg( CC112X_ATEST, &writeByte, 1); //Tempsens settings
writeByte = 0x07;
cc112xSpiWriteReg( CC112X_ATEST_MODE, &writeByte, 1); //Tempsens settings
writeByte = 0x07;
cc112xSpiWriteReg( CC112X_GBIAS1, &writeByte, 1); //Tempsens settings
writeByte = 0x01;
cc112xSpiWriteReg( CC112X_PA_IFAMP_TEST, &writeByte, 1); //Tempsens settings
//Set chip in RX
trxSpiCmdStrobe(CC112X_SRX);
//Read marcstate and wait until chip is in RX
do {
cc112xSpiReadReg(CC112X_MARCSTATE, &marcStatus, 1);
} while (marcStatus != 0x6D);


//#### Set radio in debug mode ####
// Write debug init to tx fifo
cc112xSpiWriteTxFifo(txBuffer,sizeof(txBuffer));
// Run code from FIFO
writeByte=0x01;
cc112xSpiWriteReg(CC112X_BIST, &writeByte, 1);
// Strobe IDLE
trxSpiCmdStrobe(CC112X_SIDLE);
// Set IF AMP in PD
writeByte=0x1F;
cc112xSpiWriteReg( CC112X_WOR_EVENT0_LSB, &writeByte, 1);
// Strobe SXOFF to copy command over
trxSpiCmdStrobe(CC112X_SXOFF);
//#### Radio in Debug Mode ####



//Wait until channel filter data is valid
do {
cc112xSpiReadReg(CC112X_CHFILT_I2, &RegValue, 1);
} while (!RegValue&0x08);
//Read ADC value from CHFILT_I registers
cc112xSpiReadReg(CC112X_CHFILT_I2, &RegValue, 1);
ADCValue_I = ((uint32_t)RegValue) << 16;
cc112xSpiReadReg(CC112X_CHFILT_I1, &RegValue, 1);
ADCValue_I |= (((uint32_t)RegValue) << 8) & 0x0000FF00;
cc112xSpiReadReg(CC112X_CHFILT_I0, &RegValue, 1);
ADCValue_I |= (uint32_t)(RegValue) & 0x000000FF;
//Convert ADV value to celsius
celsius = (int) ( (-b+sqrt(pow(b,2)-(4*a*(c-ADCValue_I)) ) ) / (2*a));
celsius = celsius * 1.4;
//Return degrees celsius
return celsius;
}

/*******************************************************************************
*   @fn         registerConfig
*
*   @brief      Write register settings as given by SmartRF Studio found in
*               cc112x_easy_link_reg_config.h
*
*   @param      none
*
*   @return     none
*/
static void registerConfig(void) 
{
    char writeByte;
		uint16_t i ;
    // Reset radio
    trxSpiCmdStrobe(CC112X_SRES);
    // Write registers to radio
    for(i = 0; i < (sizeof(preferredSettings)/sizeof(registerSetting_t)); i++) 
		{
			writeByte = preferredSettings[i].data;
      cc112xSpiWriteReg(preferredSettings[i].addr, &writeByte, 1);
    }
}


uint16_t UART_baud;
uint16_t divisor;
#define UART_OSCERCLK   	8000000
	/****************************************************************************************************/
/* 													 UART Initialize Function   																			 		 	*/
/****************************************************************************************************/
void USART1_Init(uint16_t baud_rate)
{
	char osr=15;
	UART_baud=baud_rate;

	//This part will be added to GPIO Init function!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		// Select "Alt 2" usage to enable UART0 on pins
	PORTA->PCR[1] = PORT_PCR_ISF_MASK|PORT_PCR_MUX(0x2);
	PORTA->PCR[2] = PORT_PCR_ISF_MASK|PORT_PCR_MUX(0x2);
	
// Turn on clock to UART0 module and select 48Mhz clock (FLL/PLL source)
  SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;
  SIM->SOPT2 &= ~SIM_SOPT2_UART0SRC_MASK;
  SIM->SOPT2 |= SIM_SOPT2_UART0SRC(1);                 //OSCERCLK selected (8MHZ Cristal)

	
	UART0->C2 &= ~(UARTLP_C2_RE_MASK | UARTLP_C2_TE_MASK |UART0_C2_RIE_MASK); 

  UART0->C2 = 0;						//disable uart 0 to change registers
  UART0->C1 = 0;
  UART0->C3 = 0;
  UART0->S2 = 0;    
		
		// Set the baud rate divisor
 	divisor = (uint16_t)(UART_OSCERCLK / ((osr+1)* baud_rate));
  UART0->C4 = osr;											//osr = 3
  UART0->BDH = (divisor >> 8) & UARTLP_BDH_SBR_MASK;
  UART0->BDL = (divisor & UARTLP_BDL_SBR_MASK);
		
//	UART0->C1 |=UART0_C1_PE_MASK; 		// parity enable

//	PTA->PCOR     |= (1UL<<5);										// Adm Recv ON && TX OFF
		
	UART0->C2 = UARTLP_C2_RE_MASK | UARTLP_C2_TE_MASK |UART0_C2_RIE_MASK;								// Enable Uart-0
	
	//Enable UART interrupt				
	NVIC_EnableIRQ(UART0_IRQn);
	NVIC_SetPriority(UART0_IRQn,2);
	
//			__asm ("cpsie i");
		
}
/****************************************************************************************************/
/*														   UART getCHAR																									      */
/****************************************************************************************************/
char UART_getchar(void )
{
//	PTA->PCOR     |= (1UL<<5);										// Adm Recv ON && TX OFF
	while (!(UART0->S1 & UART0_S1_RDRF_MASK));   //  Wait until character has been received 
	return UART0->D ;															//Recieve Char
}
/****************************************************************************************************/
/*  															UART putCHAR        																	            */
/****************************************************************************************************/
char UART_putchar(char Udata )
{
//	PTA->PSOR     |= (1UL<<5);										// Adm Recv OFF && TX ON
//	DelayUs(20);
	/* Wait until space is available in the FIFO */
	//while(!(UART0->S1 & UART0_S1_TDRE_MASK) && !(UART0->S1 & UART_S1_TC_MASK));
	while(!(UART0->S1 & UART0_S1_TDRE_MASK));

	while(!(UART0->S1 & UART_S1_TC_MASK)); 	
//  Delay(10);
	UART0->D = Udata;
//		DelayUs(20);
//		PTA->PCOR     |= (1UL<<5);										// Adm Recv ONN && TX OFF
//	return Udata;
		return 0;	  
}
/****************************************************************************************************/
/* 															  UART SEND Func        																	          */
/****************************************************************************************************/
void UART_Send(char* DATA, char datasize)
{
	int j;
//	PTA->PSOR     |= (1UL<<5);										// Adm Recv OFF && TX ON
//	Delay(1);
//	__disable_irq();
	for(j=0; j<datasize; j++)
		{
			UART_putchar(DATA[j]);
		}
//	PTA->PCOR     |= (1UL<<5);										// Adm Recv ON && TX OFF 
//	__enable_irq();
}
/****************************************************************************************************/
/*														  UART RECIEVE Func     																	            */
/****************************************************************************************************/
void UART_Recv(char* DATA, int datasize)
{
	int j;
	//	UART_RX_clr();
	//	PTA->PCOR     |= (1UL<<5);										// Adm Recv ON && TX OFF
	
//	__disable_irq();
			 
	for (j=0; j<datasize; j++)
	{
		DATA[j]=UART_getchar();
	}
	//	PTA->PCOR     |= (1UL<<5);										// Adm Recv ON && TX OFF
//	__enable_irq();
}		 
	
/************************************************************************************************************************************************************/
/**																	retarget		data write uart via printf							*******************************************************************************************/
/************************************************************************************************************************************************************/
struct __FILE
{
  int handle;
  /* Whatever you require here. If the only file you are using is */
  /* standard output using printf() for debugging, no file handling */
  /* is required. */
};


FILE __stdout;

int fputc(int ch, FILE *f) 
{
	//FPTA->PSOR     |= (1UL<<5);										// Adm Recv OFF && TX ON
  //	UART_RX_clr(UARTcount); // clear rx buffer so no recv data can fulfill the buffer 
  //	Clear_BUFFER();						
  /* Your implementation of fputc(). */
  UART_putchar((char) ch);
	//BAUD_Delay();																	//init ADM to prevent byte loss 
  //	Delay(35);
	//FPTA->PCOR     |= (1UL<<5);										// Adm Recv ON && TX OFF 
  return ch;
}

char UART_RX[100];
int UARTcount;
/*****************************************************************************************************/
/*												 Interrupt Routine for UART RECIEVE Func        										       */
/*****************************************************************************************************/	 
void UART0_IRQHandler(void)
{
	GREEN_ON;

//	UART_RX[UARTcount]=((UART0->D) & 0x7F);		//recv data = 8 bit we need 7 bit ignore first bit of the data
	UART_RX[UARTcount]= UART0->D;
	UARTcount++;

//	if(UARTcount >99)					// if more than 1500 chars clear buf
//	{
//		UART_RX_clr(UARTcount);
//	}
	
	GREEN_OFF;
}
/**********************************************************************
***********							 getRegisters									*****************
***********************************************************************	
	Aim  : 	read registers 
	NOTE :	testing the register write
**********************************************************************/
char reg[100];
void getReg_Test(void)
{
		reg[0] 	= TI_ReadByte(CC112X_DCFILT_CFG); //0x40
		reg[1] 	=	TI_ReadByte(CC112X_IOCFG2 );		 //0x06
		reg[2] 	=	TI_ReadByte(CC112X_IOCFG1) ;			//, 0xB0);
		reg[3] 	=	TI_ReadByte(CC112X_IOCFG0) ;//, 0x40);
		reg[4] 	=	TI_ReadByte(CC112X_SYNC_CFG1);//, 0x08);
		reg[5] 	=	TI_ReadByte(CC112X_DEVIATION_M);//, 0x48);
		reg[6] 	=	TI_ReadByte(CC112X_MODCFG_DEV_E);//, 0x0D);
		reg[7] 	=	TI_ReadByte(CC112X_DCFILT_CFG);//, 0x1C);
		reg[8] 	=	TI_ReadByte(CC112X_IQIC);//, 0x00);
		reg[9] 	=	TI_ReadByte(CC112X_CHAN_BW);//, 0x02);
		reg[10] =	TI_ReadByte(CC112X_SYMBOL_RATE2);//, 0x73);
		reg[11] =	TI_ReadByte(CC112X_AGC_REF);//, 0x36);
		reg[12] =	TI_ReadByte(CC112X_AGC_CS_THR);//, 0x19);
		reg[13] =	TI_ReadByte(CC112X_AGC_CFG1);//, 0x89);
		reg[14] =	TI_ReadByte(CC112X_AGC_CFG0);//, 0xCF);
		reg[15] =	TI_ReadByte(CC112X_FIFO_CFG);//, 0x00);
		reg[16] =	TI_ReadByte(CC112X_SETTLING_CFG);//, 0x03);
		reg[17] =	TI_ReadByte(CC112X_FS_CFG);//, 0x12);
		reg[18] =	TI_ReadByte(CC112X_PKT_CFG0);//, 0x20); 					//infinite packet length
		reg[19] =	TI_ReadByte(CC112X_PA_CFG0);//, 0x7B);
		reg[20] =	TI_ReadByte(CC112X_PKT_LEN);//, 0xFF);
}
/**********************************************************************
***********							 setRegisters									*****************
***********************************************************************	
	Aim  : Set default registers 
	NOTE :	
**********************************************************************/
void setRegisters(void) 
{
		TI_WriteByte(CC112X_DCFILT_CFG,0x40);
		TI_WriteByte(CC112X_IOCFG2, 0x06);
		TI_WriteByte(CC112X_IOCFG1, 0xB0);
		TI_WriteByte(CC112X_IOCFG0, 0x40);
		getReg_Test();
		TI_WriteByte(CC112X_SYNC_CFG1, 0x08);
		TI_WriteByte(CC112X_DEVIATION_M, 0x48);
		TI_WriteByte(CC112X_MODCFG_DEV_E, 0x0D);
		getReg_Test();
		TI_WriteByte(CC112X_DCFILT_CFG, 0x1C);
		getReg_Test();
		TI_WriteByte(CC112X_IQIC, 0x00);
		TI_WriteByte(CC112X_CHAN_BW, 0x02);
		getReg_Test();
		TI_WriteByte(CC112X_SYMBOL_RATE2, 0x73);
		TI_WriteByte(CC112X_AGC_REF, 0x36);
		TI_WriteByte(CC112X_AGC_CS_THR, 0x19);
		TI_WriteByte(CC112X_AGC_CFG1, 0x89);
		TI_WriteByte(CC112X_AGC_CFG0, 0xCF);
		TI_WriteByte(CC112X_FIFO_CFG, 0x00);
		TI_WriteByte(CC112X_SETTLING_CFG, 0x03);
		TI_WriteByte(CC112X_FS_CFG, 0x12);
		TI_WriteByte(CC112X_PKT_CFG0, 0x20); 					//infinite packet length
		TI_WriteByte(CC112X_PA_CFG0, 0x7B);
		TI_WriteByte(CC112X_PKT_LEN, 0xFF);
		/******************************************************
		extended registers are written here
		*********************************************************/
		config = 0x00; TI_Write_brst(CC112X_IF_MIX_CFG,&config,1);
		config = 0x22; TI_Write_brst(CC112X_FREQOFF_CFG,&config,1);
		config = 0x6C; TI_Write_brst(CC112X_FREQ2,&config,1);
		config = 0x80; TI_Write_brst(CC112X_FREQ1,&config,1);
		config = 0x00; TI_Write_brst(CC112X_FS_DIG1,&config,1);
		config = 0x5F; TI_Write_brst(CC112X_FS_DIG0,&config,1);
		config = 0x40; TI_Write_brst(CC112X_FS_CAL1,&config,1);
		config = 0x0E; TI_Write_brst(CC112X_FS_CAL0,&config,1);
		config = 0x03; TI_Write_brst(CC112X_FS_DIVTWO,&config,1);
		config = 0x33; TI_Write_brst(CC112X_FS_DSM0,&config,1);
		config = 0x17; TI_Write_brst(CC112X_FS_DVC0,&config,1);
		config = 0x50; TI_Write_brst(CC112X_FS_PFD,&config,1);
		config = 0x6E; TI_Write_brst(CC112X_FS_PRE,&config,1);
		config = 0x14; TI_Write_brst(CC112X_FS_REG_DIV_CML,&config,1);
		config = 0xAC; TI_Write_brst(CC112X_FS_SPARE,&config,1);
		config = 0xB4; TI_Write_brst(CC112X_FS_VCO0,&config,1);
		config = 0x03; TI_Write_brst(CC112X_LNA,&config,1);
		config = 0x0E; TI_Write_brst(CC112X_XOSC5,&config,1);
		config = 0x03; TI_Write_brst(CC112X_XOSC1,&config,1);
		config = 0x10; TI_Write_brst(CC112X_PKT_CFG2,&config,1);
}

/**********************************************************************
***********							 Test Program						***********************
***********************************************************************	
	Aim  : Packet Test 
	NOTE : message on the air (test)
**********************************************************************/
char buf[7];							//message buffer
typedef  struct 
{
	char Addr;
	char Priority;
	char Direction;
	char RSSI;
	char WakeUpTime;
	char Temperature;
	char AlarmFlags;
}MSG;
	
  MSG* pMSG; 		//buffer pointer
	char WriteByte;
	char testo;
/**********************************************************************
***********							 Main Program						***********************
***********************************************************************	
	Aim  : main program
	NOTE :
**********************************************************************/
int main (void) 
{
	
  SystemCoreClockUpdate();
//	SysTick_Config(10000);																					//turn SysTick timer on
	SysTick_Config(SystemCoreClock/992);															// 1ms SysTicks
	Delay(100);																												//wait for system stabilization
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTB_MASK;      	//Port-D-B clock ON 
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
	Delay(0x500);                     																//delay  
	LED_Init();           																						//Initialize the LEDs  
	Delay(100);
//	USART1_Init(9600);
	Delay(100);
	spi_init();																												// SPI-1 init 
	Delay(10);
	
	Delay(0x100);																											//delay
	hede=PTD->PDIR;
	/*TEST*/ 
//	TI_Init();
	TI_HW_Reset();
	Delay(0x1000);													//delay for logic analyzer
	hede=PTD->PDIR;
	

/**********************************************************************
***********							 Test Area					***********************
***********************************************************************/
	pMSG = (MSG*)buf;            //buffer selection
	pMSG->Addr = 0xAB;
	pMSG->Priority = 0x01;
	pMSG->Direction = 0x02;
	pMSG->RSSI = 0xDF;
	pMSG->WakeUpTime = 0x11;
	pMSG->Temperature = 0x23;
	pMSG->AlarmFlags = 0xCC;
	
//	TI_WriteByte(CC112X_IOCFG3,0x87);
//	test[5]=TI_ReadByte(CC112X_IOCFG3);
//	TI_Command(CC112X_SRES);				//sofware reset 
//	
//	test[6]=TI_ReadByte(CC112X_IOCFG3);
//	TI_WriteByte(CC112X_IOCFG3,0x87);
//	test[7]=TI_ReadByte(CC112X_IOCFG3);
//	
//	TI_Write_brst(CC112X_BURST_TXFIFO,toto,20);
//	Delay(100);
//	TI_Read_brst(CC112X_RSSI1,got,1);
	
//Delay(100);
//TI_Command(CC112X_SRES);				//sofware reset 
	
//	setRegisters();
//	getReg_Test();
//		TI_WriteByte(CC112X_DCFILT_CFG,0x40);
//		TI_WriteByte(CC112X_IOCFG2, 0x06);
//		TI_WriteByte(CC112X_IOCFG1, 0xB0);
//		TI_WriteByte(CC112X_IOCFG0, 0x40);
	
//	test[0] = getCelcius();	 //Read temp sensor TEST			
//	

/*TEST*/
// ************************************************************************************************
/*****************SPI TEST ******** ********  */
//	WriteByte = 0x40;
//	toto[0] = cc112xSpiReadReg( CC112X_IF_ADC1,&testo, 1);
//	toto[1] = cc112xSpiWriteReg( CC112X_IF_ADC1, &WriteByte, 1); //Tempsens settings, bit 6 high
//	toto[2] = cc112xSpiReadReg( CC112X_IF_ADC1,&testo, 1);
//	trxSpiCmdStrobe(CC112X_SRES);
//	toto[3] = cc112xSpiReadReg( CC112X_IF_ADC1,&testo, 1);
////		
//	WriteByte = 0x00;
//	toto[4] = cc112xSpiWriteReg(CC112X_FS_VCO2, &WriteByte, 1);
//	toto[5] = cc112xSpiReadReg(CC112X_FS_VCO2, &testo, 1);
/*****************SPI TEST ******** ****************  */
// ************************************************************************************************
//Delay(0x2000);
//	asde = 0;
	
///*TEST*/	
//	WriteByte = tempRead(); // TEST read temperature from CC1120
//	if (WriteByte <= 25)
//	{
//		YELLOW_OFF;	GREEN_OFF;	RED_OFF;
//		GREEN_ON;
//		Delay(1000); 
//	}
	
	IRQ_Init();										// Initialize IRQ
	
	WriteByte = tempRead(); // TEST read temperature from CC1120
	
	
	registerConfig();
	Delay(10);
	
	UARTcount = 0 ;

	
	
	
//	got[0]= UART0->C1;
//	got[1]= UART0->C2;
//	
//	UART_putchar(0xAB);
//	printf("ASDE");
//	Delay(1000);
//	printf("ASDE");
//	Delay(1000);
//  	printf("ASDE");
//	Delay(1000);
//	
	YELLOW_OFF;	GREEN_OFF;	RED_OFF;
	
	while(1)
	{
		
		
		runRX();
		Delay(2000); 
		RED_OFF;
		
		
		// Turn on leds 1 by 1 
//	YELLOW_ON; Delay(1000);	GREEN_ON; Delay(1000);	
//	RED_ON; Delay(1000); 
//	//Turn off leds
//	YELLOW_OFF;	GREEN_OFF;	RED_OFF;
//		SpiStart();
//		hede=PTD->PDIR;
//		DelayUs(0x1000);
//		SPI_Send(0x10);
//		x=SPI_Send(0x00);
//		x=SPI_Send(0x00);
//		SpiStop();
//		Delay(0x50);
		
//		SpiStart();
//		SPI_Send(0x10);
//		x=SPI_Send(0x00);
//		SpiStop();
//		hede=PTD->PDIR;
//		
//		SpiStop();
////		YELLOW_ON; Delay(1000);	GREEN_ON; Delay(1000);	RED_ON; Delay(1000);
////		TI_WriteByte(CC112X_IOCFG3,0x87);
////		test[2]=TI_ReadByte(CC112X_IOCFG3);
////		YELLOW_OFF;	GREEN_OFF;	RED_OFF;
//		
//	PTD->PCOR |= (1UL<<4);                             	  // CS=0, SPI start
//	
//	hede=PTD->PDIR;

////		test[0]=SPI_Send(0x10);
//	while((PTD->PDIR & 0x80)!= 0);												//Wait for CHIP_RDYn signal
//	
//	SPI_Send(0x8F);																//send the adress and get the status byte
//	test[2]= SPI_Send(0x00);															//read the adress
////		test[1]=SPI_Send(0x00);
////		test[2]=SPI_Send(0x10);
////		test[3]=SPI_Send(0x00);
////		SPI_Send(0x10);
//	
//	PTD->PSOR |= (1UL<<4);                             	  // CS=0, SPI stop
	}
}
