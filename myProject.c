// MyProject.c
// Madhura Ashok Ambre
//UTA ID : 1001150147
//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include "tm4c123gh6pm.h"

#define RED_LED          (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4))) //PF1
#define GREEN_LED        (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4))) //PF3
#define BLUE_LED         (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4))) //PF2

#define DIP_SWITCH_0     (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 4*4))) //PB4 -LSB
#define DIP_SWITCH_1     (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4))) //PA6
#define DIP_SWITCH_2     (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4))) //PA7
#define DIP_SWITCH_3     (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 0*4))) //PD0
#define DIP_SWITCH_4     (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 1*4))) //PD1
#define DIP_SWITCH_5     (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 2*4))) //PD2
#define DIP_SWITCH_6     (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 3*4))) //PD3
#define DIP_SWITCH_7     (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 1*4))) //PE1
#define DIP_SWITCH_8     (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 2*4))) //PE2
#define DIP_SWITCH_9     (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 3*4))) //PE3 -MSB

#define DATA_EN          (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4))) //PC6
#define LED              (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 5*4))) //PB5
#define U1TX             (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 5*4))) //PC5


//-----------------------------------------------------------------------------
// Global Declarations
//-----------------------------------------------------------------------------

#define   maxLength    50
uint16_t dipSwitch;
uint16_t mask,updateAdd;
char inStr[50],alphaStr[10],addStr[20],valueStr[20],setadd[20],setval[20],inChar;
uint16_t dmx512[512],rxData[512],cmdLength,value;
uint16_t tx_phase=0, rx_phase, maxadd = 10;
int addrPos = 0,valuePos = 0,add,numFields=0,c,Txon=1,mode_Flag,poll_req;
int RED_LED_TIMEOUT=0, GREEN_LED_TIMEOUT=0, BLUE_LED_TIMEOUT=0;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------


void readMode()
{
	if(DIP_SWITCH_9==0 && Txon==1)
		mode_Flag=0;   //mode_Flag= 0 means Transmit
	if(DIP_SWITCH_9==1 || Txon==0)
		mode_Flag=1;   //mode_Flag=1 means Receive
}


// Initialize Hardware
void initHw()
{
	// Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S)
    	    		   | SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_2;

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO ports A,B,D,E and F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOC |SYSCTL_RCGC2_GPIOD | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOF;

    // Configure LED and pushbutton pins
	GPIO_PORTF_DIR_R = 0x0E;  // bits 1,2 and 3 are outputs, other pins are inputs
	GPIO_PORTF_DR2R_R = 0x0E; // set drive strength to 2mA (not needed since default configuration -- for clarity)
	GPIO_PORTF_DEN_R = 0x1E;  // enable LEDs and pushbuttons
	GPIO_PORTF_PUR_R = 0x10;  // enable internal pull-up for push button

    // Configure DIP SWITCH PINS
   GPIO_PORTA_DEN_R |= 0xC0;
   GPIO_PORTA_PUR_R |= 0xC0;
   GPIO_PORTB_DEN_R |= 0x30;
   GPIO_PORTB_DR2R_R |= 0x20;
   GPIO_PORTB_DIR_R |= 0x20;
   GPIO_PORTB_PUR_R |= 0x10;
   GPIO_PORTD_DEN_R |= 0x0F;
   GPIO_PORTD_PUR_R |= 0x0F;
   GPIO_PORTE_DEN_R |= 0x0E;
   GPIO_PORTE_PUR_R |= 0x0E;


   //readDipSwitch();
   readMode();

    // Configure UART0 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/o 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module
    GPIO_PORTC_DIR_R |= 0x40;
    GPIO_PORTC_DEN_R |= 0x40;

    // Configure UART1 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;         // turn-on UART1, leave other uarts in same status
	GPIO_PORTC_DEN_R |= 0x30;                        // default, added for clarity

	GPIO_PORTC_AFSEL_R |= 0x30;                       // default, added for clarity
	GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC5_U1TX | GPIO_PCTL_PC4_U1RX;

    // Configure UART1 to 250000 baud, 8N2 format (must be 3 clocks from clock enable and config writes)
    UART1_CTL_R = 0;                                  // turn-off UART1 to allow safe programming
    UART1_CC_R = UART_CC_CS_SYSCLK;                   // use system clock (40 MHz)
    UART1_IBRD_R = 10;                                //for using UART
    UART1_FBRD_R = 0;                                 // round(fract(r)*64)=0
    UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_STP2; // configure for 8N2 w/o 16-level FIFO
    UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module
    if(mode_Flag==0)
    	UART1_IM_R = UART_IM_TXIM ;
    if(mode_Flag==1)
    {
    	UART1_IM_R = UART_IM_RXIM ;
        DATA_EN=0;
    }
    NVIC_EN0_R |= 1 << (INT_UART1-16);               // turn-on interrupt 22 (UART1)
    UART1_DR_R = 0x00;

    // Configure PWM module0 to drive RGB backlight
	// LED on M0PWM3 (PB5), M0PWM1b
    GPIO_PORTB_AFSEL_R |= 0x20;                      // select auxilary function for bit 5
    GPIO_PORTB_PCTL_R = GPIO_PCTL_PB5_M0PWM3;        // enable PWM on bit 5

	SYSCTL_RCGC0_R |= SYSCTL_RCGC0_PWM0;             // turn-on PWM0 module
	__asm(" NOP");                                   // wait 3 clocks
	__asm(" NOP");
	__asm(" NOP");
	SYSCTL_SRPWM_R = SYSCTL_SRPWM_R0;                // reset PWM0 module
	SYSCTL_SRPWM_R = 0;                              // leave reset state
	PWM0_1_CTL_R = 0;                                // turn-off PWM0 generator 1
	PWM0_1_GENB_R = PWM_0_GENB_ACTCMPBD_ZERO | PWM_0_GENB_ACTLOAD_ONE; 	  // output 3 on PWM0, gen 1b, cmpb
	PWM0_1_LOAD_R = 256;                             // set period to 40 MHz sys clock / 2 / 256 = 78.125 kHz
	PWM0_INVERT_R = PWM_INVERT_PWM3INV ;
	PWM0_1_CMPB_R = 0;
	PWM0_1_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM0 generator 1
	PWM0_ENABLE_R = PWM_ENABLE_PWM3EN;	             // enable output

    // Configure Timer 1 as the time base
	SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer
	TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
	TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
	TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
	TIMER1_TAILR_R = 0x00061A80;                     // set load value to 4e5 for 25ms interrupt rate
	TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
	NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
	TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer

}


// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
	while (UART0_FR_R & UART_FR_TXFF);
	UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
	int i;
    for (i = 0; i < strlen(str); i++)
	  putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
	while (UART0_FR_R & UART_FR_RXFE);
	return UART0_DR_R & 0xFF;
}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
	                                            // Approx clocks per us
	__asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*3
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             B    WMS_LOOP0");       // 1*3
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}


//char getcUart1()
//{
//	while (UART1_FR_R & UART_FR_RXFE);
//	return UART1_DR_R & 0xFF;
//}


void getstring()
{
    uint8_t size = 0;
    int i;

    //Clearing input buffer
    for (i = 0; i <= maxLength;i++)
    	{
    		inStr[i]= '\0';
    	}
    //--------------------------------
    while(1)
	{
       	inChar = getcUart0();
    	//GREEN_LED ^= 1;
    	if(inChar == 0x08) //backspace -> 0x08
          {
			if (size>0)
			size-=1;    //size--
		  }
    	else
          {
    		if(inChar == 0x0d) //carriage return -> 0x0d
    		  {
    			inStr[size] = '\0';
    			BLUE_LED_TIMEOUT= 25;
    			BLUE_LED = 1;
    			break;
              }
    		else
    		  {
    			inStr[size] = inChar;
    			size+=1;
    		  }
          }

    	if(size == (maxLength-2))
    	  {
    		inStr[maxLength-1] = '\0';
    		break;
    	  }
	}
}

void convertCaps()
{
	int i;
	for(i=0;i<maxLength;i++)
	{
	  if(inStr[i]>= 97 && inStr[i]<= 122)
		 inStr[i]=inStr[i]-32;
	}
}

void updateString()
{
	int i = 0, j = 0,k = 0;
	c=0;
	int addrCounter = 0, valueCounter = 0, pos = 0;
	char numStr[20],errorInStr[20];

	// Clearing Buffers and Variables
	valuePos = addrPos= 0;
	numFields = 0;
	for (i = 0; i<20;i++)
	{
		numStr[i] = addStr[i] = valueStr[i] = alphaStr[i] = errorInStr[i]='\0';
	}

	for(i = 0; i<strlen(inStr);i++)
	{
		errorInStr[i]=inStr[i];
	}

	//---------------------------------

	for(i=0; i<strlen(inStr); i++)
	{
	  if(((inStr[i]>='A' && inStr[i]<='Z') && (inStr[i+1]>='0' && inStr[i+1]<='9'))
			  | ((inStr[i]>='0' && inStr[i]<='9') && (inStr[i+1]>='A' && inStr[i+1]<='Z')))
		{
		  errorInStr[i+1]='^';
		  for(i = 0; i<15;i++)
		  	{
			   if(errorInStr[i]!='^')
		  		errorInStr[i]= ' ';
		  	}
		  putsUart0("Position of Error:");
		  putsUart0(errorInStr);
		  putsUart0("\r\n");
		  putsUart0("Error in string...Enter correct string");
		  putsUart0("\r\n");
		  c++;
		  break;
		}

	}

   if(c==0)
   {
//Find position of first alphabet of command
	for(i=0; i<strlen(inStr); i++)
      {
	   if(inStr[i]>='A' && inStr[i]<='Z')
			{
		      //alphaPos = i+1;
		      alphaStr[k] = inStr[i];
		      k++;
			}
	 }

	for(i=0; i<strlen(inStr); i++)
	  {
	    if(inStr[i]>='0' && inStr[i]<='9')
			numStr[i]=inStr[i];
	    else
            numStr[i]='\0';
      }

    k=0;
	for(i=0; i<20; i++)
	  {

	     if(numStr[i]=='\0' && (numStr[i+1]>='0' && numStr[i+1]<='9'))
			{
	    	 addrPos=i+1;
   			 addrCounter++;
			 addStr[k] = numStr[i+1];
			 k++;
			}

	     if((numStr[i+1]>='0' && numStr[i+1]<='9') && (numStr[i+2]>='0' && numStr[i+2]<='9'))
	     {
			addrCounter++;
			addStr[k] = numStr[i+2];
			k++;
	     }

	     if((numStr[i]>='0' && numStr[i]<='9') && numStr[i+1]=='\0')
	     {
			pos=i+1;
			break;
	     }
	  }


	for(j=0;j<=pos;j++)
	  {
	     numStr[j]='\0';
	  }
	 k=0;
	for(i=0; i<20; i++)
	  {

	     if((numStr[i]=='\0' && (numStr[i+1]>='0' && numStr[i+1]<='9')))
			{
	    	 valuePos=i+1;
			 valueCounter++;
			 valueStr[k] = numStr[i+1];
			 k++;
			}


	     if((numStr[i+1]>='0' && numStr[i+1]<='9') && (numStr[i+2]>='0' && numStr[i+2]<='9'))
	     {
			valueCounter++;
			valueStr[k] = numStr[i+2];
			k++;
	     }

	     if((numStr[i]>='0' && numStr[i]<='9') && numStr[i+1]=='\0')
			break;
	  }

	for(i = 0; i<20;i++)
		{
		    if(alphaStr[i]!='\0')
				{numFields++;
				break;}
		}

	for(i = 0; i<20;i++)
			{
			if(addStr[i]!='\0')
				{numFields++;
				break;}
			}

	for(i = 0; i<20;i++)
			{
			if(valueStr[i]!='\0')
				{numFields++;
				break;}
			}
	}
}

bool isCommand()
{
    int args=0;
    char str[10];

	if(strcmp(&alphaStr[0],"SET") == 0)
		{
		   int i;
		args = 2;
		   if(args==numFields-1)
		   {
			   for(i=0; i<20;i++)
			    {
			   		setadd[i]=addStr[i];
			   		setval[i]=valueStr[i];
	           }
			   add = atoi((char*)setadd);
			   value = (atoi(&setval[0]));
			  			   dmx512[add]=value;
			   return true;
		   }
		   else
		   		putsUart0("Invalid Command");

		}
	else if(strcmp(&alphaStr[0],"CLEAR") == 0)
	       { int i;
		args = 0;
			   if(args==numFields-1)
			   {
				   {
					   for(i=0;i<512;i++)
	     			      dmx512[i]= 0;

					  for(i=0;i<=1;i++)
					      {
						   setval[i]= 0;
					      }
				   }
				   putsUart0("All values set to zero");
				   putsUart0("\r\n");
				   return true;
			   }
			   else
				   putsUart0("Invalid Command");

			}
	else if(strcmp(&alphaStr[0],"GET") == 0)
		       {

		args = 1;
				   if(args==numFields-1)
				   {
		                 if(strcmp(&addStr[0],&setadd[0]) == 0)
		                	{
		                	  value = (atoi(&setval[0]));
		                	  value = dmx512[add];
		                	  putsUart0("Value: ");
		                	  sprintf(str,"%d",value);
		                	  putsUart0(str);
		                      putsUart0("\r\n");
		                	}

		                 return true;
				   }

				   else
				   putsUart0("Invalid Command");

				}

	else if(strcmp(&alphaStr[0],"ON") == 0)
		       {
		args = 0;
				   if(args==numFields-1)
				   {
					   Txon=1;
					   putsUart0("Txon flag set to 1");
					   putsUart0("\r\n");
					   return true;
				   }
				   else
				   	   putsUart0("Invalid Command");
				}
	else if(strcmp(&alphaStr[0],"OFF") == 0)
		       {
		args = 0;
				   if(args==numFields-1)
				   {
					    Txon=0;
					   	putsUart0("Txon flag set to 0");
					   	putsUart0("\r\n");
					    return true;
				   }
				   else
				   	    putsUart0("Invalid Command");
				}
	else if(strcmp(&alphaStr[0],"MAX") == 0)
		       {
		//int i,t;
		args = 1;
				   if(args==numFields-1)
				   {
					    maxadd = atoi((char*)addStr);
					    putsUart0("Maximum address :");
					   	putsUart0(addStr);
					   	putsUart0("\r\n");
					    return true;
				   }
				   else
				   	    putsUart0("Invalid Command");
				}
	else if(strcmp(&alphaStr[0],"POLL") == 0)
		       {
		args = 0;
				   if(args==numFields-1)
				   {
					    poll_req=1;
					   	putsUart0("Poll flag set to 1");
					   	putsUart0("\r\n");
				  	    return true;
				   }
				   else
				   		putsUart0("Invalid Command");

				}
	else
	{
		putsUart0("Invalid Command");
	    putsUart0("\r\n");
	}

}


void updateDevices()
{
	//readDipSwitch();
	dipSwitch = DIP_SWITCH_0 + (DIP_SWITCH_1<<1) + (DIP_SWITCH_2<<2) + (DIP_SWITCH_3<<3) + (DIP_SWITCH_4<<4) +
	       							  (DIP_SWITCH_5<<5) + (DIP_SWITCH_6<<6) + (DIP_SWITCH_7<<7) + (DIP_SWITCH_8<<8);
	       	mask = 0x1FF;
	       	dipSwitch = mask^dipSwitch;
	updateAdd = rxData[dipSwitch];
	if(updateAdd!=0)
			PWM0_1_CMPB_R = updateAdd;
	else
			PWM0_1_CMPB_R = updateAdd;
}


void Uart1Isr()
{
	readMode();
	if(mode_Flag==0 && tx_phase==0)
	{

		GPIO_PORTC_DEN_R |= 0X40;
		GPIO_PORTC_DIR_R |= 0X40;
		DATA_EN = 1;
		//Configure UART1 to 250000/3 = 83333.33 baud
		UART1_CTL_R = 0;                                 // turn-off UART1 to allow safe programming
		UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module
		UART1_IM_R = UART_IM_TXIM;                       // turn-on TX interrupt
		NVIC_EN0_R |= 1 << (INT_UART1-16);
		while (UART1_FR_R & UART_FR_TXFF);
		 	UART1_DR_R = 0;      // transmitting break & MAB
		 tx_phase += 2;
		 while (UART1_FR_R & UART_FR_BUSY);
		 UART1_CTL_R = 0;                                 // turn-off UART1 to allow safe programming
		 UART1_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
		 UART1_IBRD_R = 10;                               //for using UART
		 UART1_FBRD_R = 0;                               // round(fract(r)*64)=0
		 UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_STP2; // configure for 8N2 w/d 16-level FIFO
		 UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module
		 UART1_DR_R = 0x00;                                // Start code
		 tx_phase++;
	}

	 if(mode_Flag==0 && tx_phase>=2)
	 {
	 //int i = 0;

		 UART1_DR_R = dmx512[tx_phase-3];
		 tx_phase++;
		 if(tx_phase == maxadd+4)
		 {
			 tx_phase=0;
			 UART1_ICR_R = UART_ICR_TXIC;
		 }
		 UART1_DR_R = 0x00;
	 }

	 if(mode_Flag==1)
	 {
		uint8_t data;
	    bool FERR = UART1_DR_R & UART_DR_FE;
		data = UART1_DR_R & 0xFF;
		if( FERR && data==0)
			{
			    rx_phase = 0;
				UART1_ICR_R = UART_ICR_RXIC;
				updateDevices();

			}
		 else
		 {
			 rxData[rx_phase]= data;
			 rx_phase++;
	 	 }
	 }

}

void Timer1Isr()
{
	if(BLUE_LED == 1)
		{
			if(BLUE_LED_TIMEOUT == 0)
				BLUE_LED = 0;
			else
				BLUE_LED_TIMEOUT -= 1;
		}
	TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag
}

//void loopback()
//{
	// loopback for checking UART1 init @ 83.33kHz
//				UART1_DR_R = 'M';
//				char  x =getcUart1();
//				putcUart0(x);
//}


//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{

	 // Initialize hardware
	 initHw();
	 bool Ok;
	 char str[10];

// Toggle red LED once for 250ms
       if(RED_LED == 0)
      {
    	  RED_LED = 1;
         waitMicrosecond(250000);
          RED_LED = 0;
       }

       dipSwitch = DIP_SWITCH_0 + (DIP_SWITCH_1<<1) + (DIP_SWITCH_2<<2) + (DIP_SWITCH_3<<3) + (DIP_SWITCH_4<<4) +
       							  (DIP_SWITCH_5<<5) + (DIP_SWITCH_6<<6) + (DIP_SWITCH_7<<7) + (DIP_SWITCH_8<<8);
       	mask = 0x1FF;
       	dipSwitch = mask^dipSwitch;

	  // Display address of DIP SWITCH on UART
	  putsUart0("Address on DIP SWITCH :");
	  sprintf(str,"%d",dipSwitch);
	  putsUart0(str);
	  putsUart0("\r\n");


     while(1)
     {
		  //*********  STEP 3  *************//
    	    putsUart0("\r\n");
		    putsUart0("Enter Command:");
			getstring();
			putsUart0("Received Command :");
			putsUart0(inStr);
			putsUart0("\r\n");
			convertCaps();
			updateString();
//			putsUart0("Number of fields in the Command :");
//			sprintf(str,"%d",numFields);
//			putsUart0(str);
//			putsUart0("\r\n");
			putsUart0("Command to be processed :");
			putsUart0(alphaStr);
			putsUart0("\r\n");
//			putsUart0("Position of Address :");
//			sprintf(str,"%d",addrPos);
//			putsUart0(str);
//			putsUart0("\r\n");
//			putsUart0("Position of Value:");
//			sprintf(str,"%d",valuePos);
//			putsUart0(str);
//			putsUart0("\r\n");
			Ok = isCommand();
			if(Ok==true)
			{
				putsUart0("Ready");
	   	        putsUart0("\r\n");
			}
			//loopback();
     }
}
