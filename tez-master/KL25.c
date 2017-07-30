
//#include "main.h"
#include "MKL25Z4.h"



///**********************************************************************
//***********							 SPI_Init							*************************
//***********************************************************************	
//	Aim  : initialize SPI for Kl25
//	NOTE : 
//**********************************************************************/
//	void spi_init(void)
//{
//		  	 																								
//		int dly;
//		
//    SIM->SCGC4 |= SIM_SCGC4_SPI1_MASK;        	 								//Enable SPI1 clock  
//		
//		
//		//manual ss pin 
////		PORTD->PCR[4] |= PORT_PCR_MUX(1); 													//SS pin gpio
////		PTD->PDDR     |= (1UL<<4);																	//SS pin output PD4
//		
//		PORTD->PCR[3] |= PORT_PCR_MUX(1); 													//SS pin gpio
//		PTD->PDDR     |= (1UL<<3);																	//SS pin output PD3
//		
////		PORTA->PCR[1]   |= PORT_PCR_MUX(1); 
////		PTA->PDDR   	  |= (1UL);																	
//		
//		PORTD->PCR[2] |= PORT_PCR_MUX(1); 													//TI_Reset pin gpio
//		PTD->PDDR     |= (1UL<<2);																	//TI_Reset pin output PD2
//		
//		//PORTD->PCR[0] = PORT_PCR_MUX(0x2);           							//Set PTD4 to mux 2   (SS)
//		PORTD->PCR[5] = PORT_PCR_MUX(0x02);           							//Set PTD5 to mux 2   (clk)
//		PORTD->PCR[6] = PORT_PCR_MUX(0x02);           							//Set PTD6 to mux 2   (Mosi)
//		PORTD->PCR[7] = PORT_PCR_MUX(0x02);           							//Set PTD7 to mux 2   (Miso)
//		
//		SPI1->C1 = SPI_C1_MSTR_MASK;         											//Set SPI0 to Master   
//		SPI1->C2 = SPI_C2_MODFEN_MASK;                           	//Master SS pin acts as slave select output        
//		//SPI1->BR = (SPI_BR_SPPR(0x111) | SPI_BR_SPR(0x0100));     //Set baud rate prescale divisor to 3 & set baud rate divisor to 32 for baud rate of 15625 hz        
////		SPI1->BR |= 0x30;
//			
//			
//			SPI1->BR |= 0x43;
////			SPI1->BR |= 0x45; // test
////		SPI1->C1 |=  (1UL << 3) ; 										//SPI MOD 3
////		SPI1->C1 |=  (1UL << 2) ; 
//		
//		dly=1000;
//		while(dly--);
//		SPI1->C1 |= 0x40;																					//SPI1 Enable
//		PTD->PSOR |= (1UL<<4);
//		PTD->PSOR |= (1UL<<2);
//	}
//	
///**********************************************************************
//***********							 SPI_SEND							*************************
//***********************************************************************	
//	Aim  :  send and recieve data via SPI
//	NOTE : 
//**********************************************************************/
//char SPI_Send(char Data)
//{
//	while(!(SPI_S_SPTEF_MASK & SPI1->S));   
//	SPI1->D = Data;																								//Write Data
//	while(!(SPI_S_SPRF_MASK & SPI1->S)); 
//	return 	SPI1->D;																							//Read Data

//}


//uint16_t UART_baud;
//uint16_t divisor;
//#define UART_OSCERCLK   	8000000
///****************************************************************************************************/
///* 													 UART Initialize Function   																			 		 	*/
///****************************************************************************************************/
//void USART1_Init(uint16_t baud_rate)
//{
//	char osr=15;
//	UART_baud=baud_rate;

//	//This part will be added to GPIO Init function!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//		// Select "Alt 2" usage to enable UART0 on pins
//	PORTA->PCR[1] = PORT_PCR_ISF_MASK|PORT_PCR_MUX(0x2);
//	PORTA->PCR[2] = PORT_PCR_ISF_MASK|PORT_PCR_MUX(0x2);
//	

//	
//// Turn on clock to UART0 module and select 48Mhz clock (FLL/PLL source)
//  SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
//  SIM->SOPT2 &= ~SIM_SOPT2_UART0SRC_MASK;
//  //SIM->SOPT2 |= SIM_SOPT2_UART0SRC(1);                 //OSCERCLK selected (8MHZ Cristal)

//	
//	UART0->C2 &= ~(UARTLP_C2_RE_MASK | UARTLP_C2_TE_MASK |UART0_C2_RIE_MASK); 

//  UART0->C2 = 0;						//disable uart 0 to change registers
//  UART0->C1 = 0;
//  UART0->C3 = 0;
//  UART0->S2 = 0;    
//		
//		// Set the baud rate divisor
// 	divisor = (uint16_t)(UART_OSCERCLK / ((osr+1)* baud_rate));
//  UART0->C4 = osr;											//osr = 3
//  UART0->BDH = (divisor >> 8) & UARTLP_BDH_SBR_MASK;
//  UART0->BDL = (divisor & UARTLP_BDL_SBR_MASK);
//		
////	UART0->C1 |=UART0_C1_PE_MASK; 		// parity enable

////	PTA->PCOR     |= (1UL<<5);										// Adm Recv ON && TX OFF
//		
//	UART0->C2 = UARTLP_C2_RE_MASK | UARTLP_C2_TE_MASK |UART0_C2_RIE_MASK;								// Enable Uart-0
//	
//	//Enable UART interrupt				
//	NVIC_EnableIRQ(UART0_IRQn);
//	NVIC_SetPriority(UART0_IRQn,2);
//	
////			__asm ("cpsie i");
//		
//}
///****************************************************************************************************/
///*														   UART getCHAR																									      */
///****************************************************************************************************/
//char UART_getchar(void )
//{
////	PTA->PCOR     |= (1UL<<5);										// Adm Recv ON && TX OFF
//	while (!(UART0->S1 & UART0_S1_RDRF_MASK));   //  Wait until character has been received 
//	return UART0->D ;															//Recieve Char
//}
///****************************************************************************************************/
///*  															UART putCHAR        																	            */
///****************************************************************************************************/
//char UART_putchar(char Udata )
//{
////	PTA->PSOR     |= (1UL<<5);										// Adm Recv OFF && TX ON
////	DelayUs(20);
//	/* Wait until space is available in the FIFO */
//	//while(!(UART0->S1 & UART0_S1_TDRE_MASK) && !(UART0->S1 & UART_S1_TC_MASK));
//	while(!(UART0->S1 & UART0_S1_TDRE_MASK));

//	while(!(UART0->S1 & UART_S1_TC_MASK)); 	
////  Delay(10);
//	UART0->D = Udata;
////		DelayUs(20);
////		PTA->PCOR     |= (1UL<<5);										// Adm Recv ONN && TX OFF
////	return Udata;
//		return 0;	  
//}
///****************************************************************************************************/
///* 															  UART SEND Func        																	          */
///****************************************************************************************************/
//void UART_Send(char* DATA, char datasize)
//{
//	int j;
////	PTA->PSOR     |= (1UL<<5);										// Adm Recv OFF && TX ON
////	Delay(1);
////	__disable_irq();
//	for(j=0; j<datasize; j++)
//		{
//			UART_putchar(DATA[j]);
//		}
////	PTA->PCOR     |= (1UL<<5);										// Adm Recv ON && TX OFF 
////	__enable_irq();
//}
///****************************************************************************************************/
///*														  UART RECIEVE Func     																	            */
///****************************************************************************************************/
//void UART_Recv(char* DATA, int datasize)
//{
//	int j;
//	//	UART_RX_clr();
//	//	PTA->PCOR     |= (1UL<<5);										// Adm Recv ON && TX OFF
//	
////	__disable_irq();
//			 
//	for (j=0; j<datasize; j++)
//	{
//		DATA[j]=UART_getchar();
//	}
//	//	PTA->PCOR     |= (1UL<<5);										// Adm Recv ON && TX OFF
////	__enable_irq();
//}		 
