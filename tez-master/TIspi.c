

// will be filled by following functions from the main with function descriptions

////#include "main.h"
//#include "KL25.h"
////#include "MKL25Z4.h"                    // Device header
//#include "TIspi.h"




//char trx16BitRegAccess(uint8_t accessType, uint8_t extAddr, uint8_t regAddr, char *pData, uint8_t len);
//uint8_t trx8BitRegAccess(uint8_t accessType, uint8_t addrByte, char *pData, uint16_t len);
//static void trxReadWriteBurstSingle(uint8_t addr,char *pData,uint16_t len);
//uint8_t cc112xSpiReadReg(uint16_t addr, char *pData, uint8_t len)	;


//char testt;
//char x;
///**********************************************************************
//***********							 TI_HW_Reset							*************************
//***********************************************************************	
//	Aim  : Hardware Reset using MCU
//	NOTE :active low for 200 ns, other times high
//**********************************************************************/
//void TI_HW_Reset(void)
//{
//	PTD->PCOR |= (1UL<<2);
//	Delay(0x05);
//	PTD->PSOR |= (1UL<<2);
//}

///**********************************************************************
//***********							 TI_Init							*************************
//***********************************************************************	
//	Aim  : wait for initialization of CC1120
//	NOTE :
//**********************************************************************/
//void TI_Init(void)
//{
//	TI_HW_Reset();
//	
//	SpiStart();																										//Start SPI by CSn Low
//	wait_CHIP_RDYn; 																							//Wait for TI's christal to be stabilized	
////	while((SPI_Send(0x00) & 0x80) != 0);
////	x= SPI_Send(0x00);
//	//	Delay(0x5000);
//	SpiStop();																										//Stop SPI by CSn High
//	
//}

///**********************************************************************
//***********							 TI_Write							*************************
//***********************************************************************	
//	Aim  : write 1 byte data to chip via SPI
//	NOTE :[ R/W bit (0)] + [ Burst bit (0)] + [6 bit addres] + 8 bit data
//**********************************************************************/
//void TI_WriteByte( char addr,  char data)
//{
////int dly;
////  SpiStart();																										//Start SPI by CSn Low
////	wait_CHIP_RDYn; 																							//Wait for TI's christal to be stabilized
//	SPI_Send(addr);																							  //Send 1 byte addr and write command
//	SPI_Send(data);																								//Send 1 byte data 
////	SpiStop();	                               										//Stop SPI by CSn High
//}

///**********************************************************************
//***********							 TI_Read							*************************
//***********************************************************************	
//	Aim  : Read 1 byte data from chip via SPI
//	NOTE :[ R/W bit (1)] + [ Burst bit (0)] + [6 bit addres]
//**********************************************************************/
//char TI_ReadByte(char addr)
//{
//  char data = 0;
//	
////	SpiStart();																										//Start SPI by CSn Low
////	wait_CHIP_RDYn; 																							//Wait for TI's christal to be stabilized
//	//SPI_Send( READ_SINGLE | addr);																// R/w bit (1) + Burst bit (0)+ 6 bit addres
//	data = SPI_Send(0x00);             														// Data read (read 1byte data) via dummy write
////	SpiStop();																										//Stop SPI by CSn High
//  return data;    
//}
///**********************************************************************
//***********							 TI_Write_brst							*******************
//***********************************************************************	
//	Aim  : burst write to TI chip
//	NOTE :[ R/W bit (0)] + [ Burst bit (1)] + [6 bit addres stars with 0x2F00]
//**********************************************************************/
//int TI_Write_brst(int addr,char* buf,int len)
//{
//  int i = 0;

////	SpiStart();					                             	  	//Start SPI by CSn Low
////	wait_CHIP_RDYn; 																			//Wait for TI's christal to be stabilized
//	//SPI_Send ( WRITE_BURST | addr );											// Send the adrr
//		
//	for(i = 0; i < len; i++)              								// Burst Write the data 
//		{
//			SPI_Send (buf[i]);
//		}
//	 
////	SpiStop();																						//Stop SPI by CSn High
//	return len;  
//}
///**********************************************************************
//***********							 TI_READ_brst							*********************
//***********************************************************************	
//	Aim  : burst read to TI chip
//	NOTE :[ R/W bit (1)] + [ Burst bit (1)] + [6 bit addres]
//**********************************************************************/
//int TI_Read_brst(int addr, char* buf,int len)
//{	
//	int i = 0;

////	SpiStart();					                             	  	//Start SPI by CSn Low
////	wait_CHIP_RDYn; 																			//Wait for TI's christal to be stabilized
//	//SPI_Send ( READ_BURST | addr );			 test									// Address byte 1
//	for(i = 0; i < len; i++)                    				  // Write data in loop
//		{
//			buf[i] = SPI_Send(0x00);													//write data to buffer with size of "len"
//		}
////	SpiStop();																						//Stop SPI by CSn High
//	return len;
//}

///**********************************************************************
//***********							 Write TI_Command							*****************
//***********************************************************************	
//	Aim  : command strobe acces
//	NOTE :[ R/W bit (0)] + [ Burst bit (0)] + [6 bit addres]  
//				 No data is expected. Chip_status_Byte is returned from chip
//**********************************************************************/
//void TI_Command( char command )
//{
//	SpiStart();																								//Start SPI by CSn Low
//	wait_CHIP_RDYn; 																					//Wait for TI's christal to be stabilized
//	SPI_Send(command);																				//Send chip command
//	SpiStop();	                               								//Stop SPI by CSn High
//}

//char TI_Command_Read(char command)
//{
//	char ret;
//	SpiStart();																								//Start SPI by CSn Low
//	wait_CHIP_RDYn; 																					//Wait for TI's christal to be stabilized
//	SPI_Send(command|READ_SINGLE);														//Send chip command with READ bit
//	ret=SPI_Send(0x00);																				//send dummy byte so read status byte
//	SpiStop();	                               								//Stop SPI by CSn High
//	return ret;
//}

///**********************************************************************
//***********							 trxReadWriteBurstSingle							*******************
//***********************************************************************	
//	Aim  : TEST issue 
//	NOTE :
//**********************************************************************/
//static void trxReadWriteBurstSingle(uint8_t addr,char *pData,uint16_t len)
//{
//	
//	/* Communicate len number of bytes: if RX - the procedure sends 0x00 to push bytes from slave*/
//  if(addr&RADIO_READ_ACCESS)
//  {
//    if(addr&RADIO_BURST_ACCESS)
//    {
//     TI_Read_brst(addr,pData,len);
//    }
//    else
//    {
//     TI_ReadByte(addr);
//    }
//  }
//  else
//  {
//    if(addr&RADIO_BURST_ACCESS)
//    {
//      /* Communicate len number of bytes: if TX - the procedure doesn't overwrite pData */
//      TI_Write_brst(addr,pData,len);
//    }
//    else
//    {
//      TI_WriteByte(addr,*pData);
//    }
//  }
//  return;
//}


///**********************************************************************
//***********							 trx16BitRegAccess							*******************
//***********************************************************************	
//	Aim  : TEST issue 
//	NOTE :
//**********************************************************************/
//char trx16BitRegAccess(uint8_t accessType, uint8_t extAddr, uint8_t regAddr, char *pData, uint8_t len)
//{
//  uint8_t readValue;

//  SpiStart();					                             	  	//Start SPI by CSn Low
//  wait_CHIP_RDYn; 
//  /* send extended address byte with access type bits set */
//  readValue =SPI_Send(accessType|extAddr); //Storing chip status */
//  SPI_Send(regAddr);
//  /* Communicate len number of bytes */
//  trxReadWriteBurstSingle(accessType|extAddr,pData,len);
//  SpiStop();
//  /* return the status byte value */
//  return(readValue);
//}

///**********************************************************************
//***********							 trx8BitRegAccess							*******************
//***********************************************************************	
//	Aim  : TEST issue 
//	NOTE :
//**********************************************************************/
//uint8_t trx8BitRegAccess(uint8_t accessType, uint8_t addrByte, char *pData, uint16_t len)
//{
//  uint8_t readValue;

//  /* Pull CS_N low and wait for SO to go low before communication starts */
//  SpiStart();
//  wait_CHIP_RDYn;
//  /* send register address byte */
//  readValue = SPI_Send(accessType|addrByte);  /* Storing chip status */
////  while(!(SPI_S_SPTEF_MASK & SPI1->S));   
////	SPI1->D = (accessType|addrByte);																								//Write Data
////	while(!(SPI_S_SPRF_MASK & SPI1->S)); 
////	readValue = SPI1->D;
//	trxReadWriteBurstSingle(accessType|addrByte,pData,len);
//  SpiStop();
//  /* return the status byte value */
//  return(readValue);
//}

///******************************************************************************
// * @fn          cc112xSpiReadReg
// *
// * @brief       Read value(s) from config/status/extended radio register(s).
// *              If len  = 1: Reads a single register
// *              if len != 1: Reads len register values in burst mode 
// *
// * input parameters
// *
// * @param       addr   - address of first register to read
// * @param       *pData - pointer to data array where read bytes are saved
// * @param       len   - number of bytes to read
// *
// * output parameters
// *
// * @return      rfStatus_t
// */
//uint8_t cc112xSpiReadReg(uint16_t addr, char *pData, uint8_t len)
//{
//  uint8_t tempExt  = (uint8_t)(addr>>8);
//  uint8_t tempAddr = (uint8_t)(addr & 0x00FF);
//  uint8_t rc;
//  
//  /* Checking if this is a FIFO access -> returns chip not ready  */
//  if((CC112X_SINGLE_TXFIFO<=tempAddr)&&(tempExt==0)) return STATUS_CHIP_RDYn_BM;
//  
//  /* Decide what register space is accessed */
//  if(!tempExt)
//  {
//    rc = trx8BitRegAccess((RADIO_BURST_ACCESS|RADIO_READ_ACCESS),tempAddr,pData,len);
//  }
//  else if (tempExt == 0x2F)
//  {
//    rc = trx16BitRegAccess((RADIO_BURST_ACCESS|RADIO_READ_ACCESS),tempExt,tempAddr,pData,len);
//  }
//  return (rc);
//}

///*******************************************************************************
// * @fn          trxSpiCmdStrobe
// *
// * @brief       Send command strobe to the radio. Returns status byte read
// *              during transfer of command strobe. Validation of provided
// *              is not done. Function assumes chip is ready.
// *
// * input parameters
// *
// * @param       cmd - command strobe
// *
// * output parameters
// *
// * @return      status byte
// */
//uint8_t trxSpiCmdStrobe(uint8_t cmd)
//{
//    uint8_t rc;
//    SpiStart();
//		wait_CHIP_RDYn;
//    rc = SPI_Send(cmd);
//    SpiStop();
//    return(rc);
//}

///******************************************************************************
// * @fn          cc112xSpiWriteReg
// *
// * @brief       Write value(s) to config/status/extended radio register(s).
// *              If len  = 1: Writes a single register
// *              if len  > 1: Writes len register values in burst mode 
// *
// * input parameters
// *
// * @param       addr   - address of first register to write
// * @param       *pData - pointer to data array that holds bytes to be written
// * @param       len    - number of bytes to write
// *
// * output parameters
// *
// * @return      rfStatus_t
// */
//uint8_t cc112xSpiWriteReg(uint16_t addr, char *pData, uint8_t len)
//{
//  uint8_t tempExt  = (uint8_t)(addr>>8);
//  uint8_t tempAddr = (uint8_t)(addr & 0x00FF);
//  uint8_t rc;
//  
//  /* Checking if this is a FIFO access -> returns chip not ready  */
//  if((CC112X_SINGLE_TXFIFO<=tempAddr)&&(tempExt==0)) return STATUS_CHIP_RDYn_BM;
//  
//  /* Decide what register space is accessed */
//  if(!tempExt)
//  {
//    rc = trx8BitRegAccess((RADIO_BURST_ACCESS|RADIO_WRITE_ACCESS),tempAddr,pData,len);
//  }
//  else if (tempExt == 0x2F)
//  {
//    rc = trx16BitRegAccess((RADIO_BURST_ACCESS|RADIO_WRITE_ACCESS),tempExt,tempAddr,pData,len);
//  }
//  return (rc);
//}
///*******************************************************************************
// * @fn          cc112xSpiWriteTxFifo
// *
// * @brief       Write pData to radio transmit FIFO.
// *
// * input parameters
// *
// * @param       *pData - pointer to data array that is written to TX FIFO
// * @param       len    - Length of data array to be written
// *
// * output parameters
// *
// * @return      rfStatus_t (uint8_t)
// */
//uint8_t cc112xSpiWriteTxFifo(char *pData, uint8_t len)
//{
//  uint8_t rc;
//  rc = trx8BitRegAccess(0x00,CC112X_BURST_TXFIFO, pData, len);
//  return (rc);
//}
///*******************************************************************************
// * @fn          cc112xSpiReadRxFifo
// *
// * @brief       Reads RX FIFO values to pData array
// *
// * input parameters
// *
// * @param       *pData - pointer to data array where RX FIFO bytes are saved
// * @param       len    - number of bytes to read from the RX FIFO
// *
// * output parameters
// *
// * @return      rfStatus_t
// */
//uint8_t cc112xSpiReadRxFifo(char * pData, uint8_t len)
//{
//  uint8_t rc;
//  rc = trx8BitRegAccess(0x00,CC112X_BURST_RXFIFO, pData, len);
//  return (rc);
//}

///******************************************************************************
// * @fn      cc112xGetTxStatus(void)
// *          
// * @brief   This function transmits a No Operation Strobe (SNOP) to get the 
// *          status of the radio and the number of free bytes in the TX FIFO.
// *          
// *          Status byte:
// *          
// *          ---------------------------------------------------------------------------
// *          |          |            |                                                 |
// *          | CHIP_RDY | STATE[2:0] | FIFO_BYTES_AVAILABLE (free bytes in the TX FIFO |
// *          |          |            |                                                 |
// *          ---------------------------------------------------------------------------
// *
// *
// * input parameters
// *
// * @param   none
// *
// * output parameters
// *         
// * @return  rfStatus_t 
// *
// */
//uint8_t cc112xGetTxStatus(void)
//{
//    return(trxSpiCmdStrobe(CC112X_SNOP));
//}

///******************************************************************************
// *
// *  @fn       cc112xGetRxStatus(void)
// *
// *  @brief   
// *            This function transmits a No Operation Strobe (SNOP) with the 
// *            read bit set to get the status of the radio and the number of 
// *            available bytes in the RXFIFO.
// *            
// *            Status byte:
// *            
// *            --------------------------------------------------------------------------------
// *            |          |            |                                                      |
// *            | CHIP_RDY | STATE[2:0] | FIFO_BYTES_AVAILABLE (available bytes in the RX FIFO |
// *            |          |            |                                                      |
// *            --------------------------------------------------------------------------------
// *
// *
// * input parameters
// *
// * @param     none
// *
// * output parameters
// *         
// * @return    rfStatus_t 
// *
// */
//uint8_t cc112xGetRxStatus(void)
//{
//    return(trxSpiCmdStrobe(CC112X_SNOP | RADIO_READ_ACCESS));
//}

///*******************************************************************************
//*   @fn         manualCalibration
//*
//*   @brief      Calibrates radio according to CC112x errata
//*
//*   @param      none
//*
//*   @return     none
//*/
//#define VCDAC_START_OFFSET 2
//#define FS_VCO2_INDEX 0
//#define FS_VCO4_INDEX 1
//#define FS_CHP_INDEX 2

//static void RX_manualCalibration(void) 
//{

//    char original_fs_cal2;
//    char calResults_for_vcdac_start_high[3];
//    char calResults_for_vcdac_start_mid[3];
//    char marcstate;
//    char writeByte;

//    // 1) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
//    writeByte = 0x00;
//    cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);

//    // 2) Start with high VCDAC (original VCDAC_START + 2):
//    cc112xSpiReadReg(CC112X_FS_CAL2, &original_fs_cal2, 1);
//    writeByte = original_fs_cal2 + VCDAC_START_OFFSET;
//    cc112xSpiWriteReg(CC112X_FS_CAL2, &writeByte, 1);

//    // 3) Calibrate and wait for calibration to be done
//    //   (radio back in IDLE state)
//    trxSpiCmdStrobe(CC112X_SCAL);

//    do {
//        cc112xSpiReadReg(CC112X_MARCSTATE, &marcstate, 1);
//    } while (marcstate != 0x41);

//    // 4) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained with
//    //    high VCDAC_START value
//    cc112xSpiReadReg(CC112X_FS_VCO2,
//                     &calResults_for_vcdac_start_high[FS_VCO2_INDEX], 1);
//    cc112xSpiReadReg(CC112X_FS_VCO4,
//                     &calResults_for_vcdac_start_high[FS_VCO4_INDEX], 1);
//    cc112xSpiReadReg(CC112X_FS_CHP,
//                     &calResults_for_vcdac_start_high[FS_CHP_INDEX], 1);

//    // 5) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
//    writeByte = 0x00;
//    cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);

//    // 6) Continue with mid VCDAC (original VCDAC_START):
//    writeByte = original_fs_cal2;
//    cc112xSpiWriteReg(CC112X_FS_CAL2, &writeByte, 1);

//    // 7) Calibrate and wait for calibration to be done
//    //   (radio back in IDLE state)
//    trxSpiCmdStrobe(CC112X_SCAL);

//    do {
//        cc112xSpiReadReg(CC112X_MARCSTATE, &marcstate, 1);
//    } while (marcstate != 0x41);

//    // 8) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained
//    //    with mid VCDAC_START value
//    cc112xSpiReadReg(CC112X_FS_VCO2,
//                     &calResults_for_vcdac_start_mid[FS_VCO2_INDEX], 1);
//    cc112xSpiReadReg(CC112X_FS_VCO4,
//                     &calResults_for_vcdac_start_mid[FS_VCO4_INDEX], 1);
//    cc112xSpiReadReg(CC112X_FS_CHP,
//                     &calResults_for_vcdac_start_mid[FS_CHP_INDEX], 1);

//    // 9) Write back highest FS_VCO2 and corresponding FS_VCO
//    //    and FS_CHP result
//    if (calResults_for_vcdac_start_high[FS_VCO2_INDEX] >
//        calResults_for_vcdac_start_mid[FS_VCO2_INDEX]) {
//        writeByte = calResults_for_vcdac_start_high[FS_VCO2_INDEX];
//        cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);
//        writeByte = calResults_for_vcdac_start_high[FS_VCO4_INDEX];
//        cc112xSpiWriteReg(CC112X_FS_VCO4, &writeByte, 1);
//        writeByte = calResults_for_vcdac_start_high[FS_CHP_INDEX];
//        cc112xSpiWriteReg(CC112X_FS_CHP, &writeByte, 1);
//    } else {
//        writeByte = calResults_for_vcdac_start_mid[FS_VCO2_INDEX];
//        cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);
//        writeByte = calResults_for_vcdac_start_mid[FS_VCO4_INDEX];
//        cc112xSpiWriteReg(CC112X_FS_VCO4, &writeByte, 1);
//        writeByte = calResults_for_vcdac_start_mid[FS_CHP_INDEX];
//        cc112xSpiWriteReg(CC112X_FS_CHP, &writeByte, 1);
//    }
//}

///*******************************************************************************
//*   @fn         runRX
//*
//*   @brief      Puts radio in RX and waits for packets. Function assumes
//*               that status bytes are appended in the RX_FIFO
//*               Update packet counter and display for each packet received.
//*		
//*		@Note 			1. GPIO2 has to be set up with GPIO2_CFG = 0x06-->   PKT_SYNC_RXTX for correct interupt
////              2. Packet engine has to be set up with status bytes enabled PKT_CFG1.APPEND_STATUS = 1
//*   @param      none
//*
//*   @return     none
//*********************************************************************************/
//static void runRX(void) 
//{
//	
//	char rxBuffer[128] = {0};
//  char rxBytes;
//  char marcState;
//	
//	// Calibrate radio according to errata
//  RX_manualCalibration();
//	
//	// Set radio in RX
//  trxSpiCmdStrobe(CC112X_SRX);
//	
//	
//	// Wait for packet received interrupt
//	if(packetSemaphore == ISR_ACTION_REQUIRED) 
//		{
//			// Read number of bytes in RX FIFO
//			cc112xSpiReadReg(CC112X_NUM_RXBYTES, &rxBytes, 1);

//			// Check that we have bytes in FIFO
//			if(rxBytes != 0) 
//				{
//					// Read MARCSTATE to check for RX FIFO error
//					cc112xSpiReadReg(CC112X_MARCSTATE, &marcState, 1);

//					// Mask out MARCSTATE bits and check if we have a RX FIFO error
//					if((marcState & 0x1F) == RX_FIFO_ERROR) 
//						{
//						// Flush RX FIFO
//						trxSpiCmdStrobe(CC112X_SFRX);
//						} 
//						else 
//						{
//							// Read n bytes from RX FIFO
//							cc112xSpiReadRxFifo(rxBuffer, rxBytes);

//							// Check CRC ok (CRC_OK: bit7 in second status byte)
//							// This assumes status bytes are appended in RX_FIFO
//							// (PKT_CFG1.APPEND_STATUS = 1)
//							// If CRC is disabled the CRC_OK field will read 1
//							if(rxBuffer[rxBytes - 1] & 0x80) 
//								{
//									// Update packet counter
//									packetCounter++;
//								}
//						 }
//				 }
//				if(rxBuffer[0] != 0)
//				{
//					GREEN_ON;
//				}
//				// Reset packet semaphore
//				packetSemaphore = ISR_IDLE;
//				// Set radio back in RX
//				trxSpiCmdStrobe(CC112X_SRX);
//    }
//}

//#define PKTLEN	30  // 1 < PKTLEN < 126

//#define GPIO3   0x04
//#define GPIO2   0x08
//#define GPIO0   0x80
///*******************************************************************************
//*   @fn         createPacket
//*
//*   @brief      This function is called before a packet is transmitted. It fills
//*               the txBuffer with a packet consisting of a length byte, two
//*               bytes packet counter and n random bytes.
//*
//*               The packet format is as follows:
//*               |--------------------------------------------------------------|
//*               |           |           |           |         |       |        |
//*               | pktLength | pktCount1 | pktCount0 | rndData |.......| rndData|
//*               |           |           |           |         |       |        |
//*               |--------------------------------------------------------------|
//*                txBuffer[0] txBuffer[1] txBuffer[2]            txBuffer[PKTLEN]
//*
//*   @param       Pointer to start of txBuffer
//*
//*   @return      none
//*/
//static void createPacket(char txBuffer[]) 
//{
//	uint8_t i;
//  txBuffer[0] = PKTLEN;                           // Length byte
//  txBuffer[1] = (uint8_t) (packetCounter >> 8);     // MSB of packetCounter
//  txBuffer[2] = (uint8_t)  packetCounter;           // LSB of packetCounter

//    // Fill rest of buffer with random bytes
//  for(i = 3; i < (PKTLEN + 1); i++) 
//	{
//		txBuffer[i] = 0xAB;
//  }
//}
///*******************************************************************************
//*   @fn         manualCalibration
//*
//*   @brief      Calibrates radio according to CC112x errata
//*
//*   @param      none
//*
//*   @return     none
//*/
//#define VCDAC_START_OFFSET 2
//#define FS_VCO2_INDEX 0
//#define FS_VCO4_INDEX 1
//#define FS_CHP_INDEX 2
//static void TX_manualCalibration(void) 
//{
//  char original_fs_cal2;
//  char calResults_for_vcdac_start_high[3];
//  char calResults_for_vcdac_start_mid[3];
//  char marcstate;
//  char writeByte;

//  // 1) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
//  writeByte = 0x00;
//  cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);

//    // 2) Start with high VCDAC (original VCDAC_START + 2):
//    cc112xSpiReadReg(CC112X_FS_CAL2, &original_fs_cal2, 1);
//    writeByte = original_fs_cal2 + VCDAC_START_OFFSET;
//    cc112xSpiWriteReg(CC112X_FS_CAL2, &writeByte, 1);

//    // 3) Calibrate and wait for calibration to be done
//    //   (radio back in IDLE state)
//    trxSpiCmdStrobe(CC112X_SCAL);

//    do {
//        cc112xSpiReadReg(CC112X_MARCSTATE, &marcstate, 1);
//    } while (marcstate != 0x41);

//    // 4) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained with 
//    //    high VCDAC_START value
//    cc112xSpiReadReg(CC112X_FS_VCO2,
//                     &calResults_for_vcdac_start_high[FS_VCO2_INDEX], 1);
//    cc112xSpiReadReg(CC112X_FS_VCO4,
//                     &calResults_for_vcdac_start_high[FS_VCO4_INDEX], 1);
//    cc112xSpiReadReg(CC112X_FS_CHP,
//                     &calResults_for_vcdac_start_high[FS_CHP_INDEX], 1);

//    // 5) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
//    writeByte = 0x00;
//    cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);

//    // 6) Continue with mid VCDAC (original VCDAC_START):
//    writeByte = original_fs_cal2;
//    cc112xSpiWriteReg(CC112X_FS_CAL2, &writeByte, 1);

//    // 7) Calibrate and wait for calibration to be done
//    //   (radio back in IDLE state)
//    trxSpiCmdStrobe(CC112X_SCAL);

//    do {
//        cc112xSpiReadReg(CC112X_MARCSTATE, &marcstate, 1);
//    } while (marcstate != 0x41);

//    // 8) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained 
//    //    with mid VCDAC_START value
//    cc112xSpiReadReg(CC112X_FS_VCO2, 
//                     &calResults_for_vcdac_start_mid[FS_VCO2_INDEX], 1);
//    cc112xSpiReadReg(CC112X_FS_VCO4,
//                     &calResults_for_vcdac_start_mid[FS_VCO4_INDEX], 1);
//    cc112xSpiReadReg(CC112X_FS_CHP,
//                     &calResults_for_vcdac_start_mid[FS_CHP_INDEX], 1);

//    // 9) Write back highest FS_VCO2 and corresponding FS_VCO
//    //    and FS_CHP result
//    if (calResults_for_vcdac_start_high[FS_VCO2_INDEX] >
//        calResults_for_vcdac_start_mid[FS_VCO2_INDEX]) {
//        writeByte = calResults_for_vcdac_start_high[FS_VCO2_INDEX];
//        cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);
//        writeByte = calResults_for_vcdac_start_high[FS_VCO4_INDEX];
//        cc112xSpiWriteReg(CC112X_FS_VCO4, &writeByte, 1);
//        writeByte = calResults_for_vcdac_start_high[FS_CHP_INDEX];
//        cc112xSpiWriteReg(CC112X_FS_CHP, &writeByte, 1);
//    } else {
//        writeByte = calResults_for_vcdac_start_mid[FS_VCO2_INDEX];
//        cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);
//        writeByte = calResults_for_vcdac_start_mid[FS_VCO4_INDEX];
//        cc112xSpiWriteReg(CC112X_FS_VCO4, &writeByte, 1);
//        writeByte = calResults_for_vcdac_start_mid[FS_CHP_INDEX];
//        cc112xSpiWriteReg(CC112X_FS_CHP, &writeByte, 1);
//    }
//}
///*******************************************************************************
//*   @fn         runTX
//*
//*   @brief      Continuously sends packets on button push until button is pushed
//*               again. After the radio has gone into TX the function waits for
//*               interrupt that packet has been sent. Updates packet counter and
//*               display for each packet sent.
//*
//*   @param      none
//*
//*   @return    none
//*/
//static void runTX(void) 
//{
//	// Initialize packet buffer of size PKTLEN + 1
// // char txBuffer[PKTLEN+1] = {0}; "AMK POKUMANI";
//	char txBuffer[PKTLEN+1] = "AMK POKUMANI";
//	
//	// Calibrate radio according to errata
//   TX_manualCalibration();

//	// Update packet counter
//  packetCounter++;

//  // Create a random packet with PKTLEN + 2 byte packet
//  // counter + n x random bytes
//  createPacket(txBuffer);

//  // Write packet to TX FIFO
//  cc112xSpiWriteTxFifo(txBuffer, sizeof(txBuffer));

//  // Strobe TX to send packet
//  trxSpiCmdStrobe(CC112X_STX);

//  // Wait for interrupt that packet has been sent.
//  // (Assumes the GPIO connected to the radioRxTxISR function is
//  // set to GPIOx_CFG = 0x06)
//  while(packetSemaphore != ISR_ACTION_REQUIRED);
//  // Clear semaphore flag
//  packetSemaphore = ISR_IDLE;
// 
//}
