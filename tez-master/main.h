
#include "MKL25Z4.h"                    // Device header

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
//char test[22];
//char C[4];
//int  hede;
//char config;
//char toto[22]="AMK POKUMANI";
//int  packetCounter = 0;
//char got[22];
//char packetSemaphore;

//__INLINE static void Delay (uint32_t dlyTicks);
//void TI_HW_Reset(void);
//void TI_Init(void);
//void TI_WriteByte( char addr,  char data);
//char TI_ReadByte(char addr);
//int TI_Write_brst(int addr,char* buf,int len);
//int TI_Read_brst(int addr, char* buf,int len);
//void TI_Command( char command );
//char TI_Command_Read(char command);
//static void runTX(void) ;
//static void runRX(void);
//static void trxReadWriteBurstSingle(uint8_t addr,char *pData,uint16_t len);
//char trx16BitRegAccess(uint8_t accessType, uint8_t extAddr, uint8_t regAddr, char *pData, uint8_t len);
//uint8_t trx8BitRegAccess(uint8_t accessType, uint8_t addrByte, char *pData, uint16_t len);
//uint8_t cc112xSpiReadReg(uint16_t addr, char *pData, uint8_t len);
//uint8_t trxSpiCmdStrobe(uint8_t cmd);
//uint8_t cc112xSpiWriteReg(uint16_t addr, char *pData, uint8_t len);
//uint8_t cc112xSpiWriteTxFifo(char *pData, uint8_t len);
//uint8_t cc112xSpiReadRxFifo(char * pData, uint8_t len);
//uint8_t cc112xGetTxStatus(void);
//uint8_t cc112xGetRxStatus(void);
//static void RX_manualCalibration(void);

//static void createPacket(char txBuffer[]);
//static void TX_manualCalibration(void) ;


