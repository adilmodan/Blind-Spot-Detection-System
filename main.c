#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "switch_counter_interrupt_TivaWare.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/uart.h"



void
PortFunctionInit(void)
{
    //
    // Enable Peripheral Clocks 
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Enable pin PF4 for GPIOInput
    //
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);

    //
    // Enable pin PF0 for GPIOInput
    //

    //
    //First open the lock and select the bits we want to modify in the GPIO commit register.
    //
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) = 0x1;

    //
    //Now modify the configuration of the pins that we unlocked.
    //
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0);
		GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
		GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
			GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);
				
		//Enable pull-up on PF4 and PF0
		GPIO_PORTF_PUR_R |= 0x11; 

}

void PLL_Init(void){

  // 0) Use RCC2
  SYSCTL_RCC2_R |=  0x80000000;  // USERCC2

  // 1) bypass PLL while initializing
  SYSCTL_RCC2_R |=  0x00000800;  // BYPASS2, PLL bypass

  // 2) select the crystal value and oscillator source
  SYSCTL_RCC_R = (SYSCTL_RCC_R &~0x000007C0)   // clear XTAL field, bits 10-6
                 + 0x00000540;   // 10101, configure for 16 MHz crystal
  SYSCTL_RCC2_R &= ~0x00000070;  // configure for main oscillator source

  // 3) activate PLL by clearing PWRDN
  SYSCTL_RCC2_R &= ~0x00002000;

  // 4) set the desired system divider
  SYSCTL_RCC2_R |= 0x40000000;   // use 400 MHz PLL
  SYSCTL_RCC2_R = (SYSCTL_RCC2_R&~ 0x1FC00000)  // clear system clock divider
                  + (4<<22);      // configure for 80 MHz clock

  // 5) wait for the PLL to lock by polling PLLLRIS
  while((SYSCTL_RIS_R&0x00000040)==0){};  // wait for PLLRIS bit

  // 6) enable use of PLL by clearing BYPASS
  SYSCTL_RCC2_R &= ~0x00000800;

}

void Interrupt_Init(void)
{
  IntEnable(INT_GPIOF);  							// enable interrupt 30 in NVIC (GPIOF)
	IntPrioritySet(INT_GPIOF, 0x00); 		// configure GPIOF interrupt priority as 0
	GPIO_PORTF_IM_R |= 0x11;   		// arm interrupt on PF0 and PF4
	GPIO_PORTF_IS_R &= ~0x11;     // PF0 and PF4 are edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x11;   	// PF0 and PF4 not both edges trigger 
  GPIO_PORTF_IEV_R &= ~0x11;  	// PF0 and PF4 falling edge event
	
}

//------------UART_InCharNonBlocking------------
// Get oldest serial port input and return immediately
// if there is no data.
// Input: none
// Output: ASCII code for key typed or 0 if no character
unsigned char UART_InCharNonBlocking(void){
  if((UART1_FR_R&UART_FR_RXFE) == 0){
    return((unsigned char)(UART1_DR_R&0xFF));
  } else{
    return 0;
  }
}

//------------UART_InChar------------
// Wait for new serial port input
// Input: none
// Output: ASCII code for key typed

unsigned char UART_InChar(void)
{while((UART1_FR_R&UART_FR_RXFE) != 0);
return((unsigned char)(UART1_DR_R&0xFF));
}

//------------UART_OutChar------------
// Output 8-bit to serial port
// Input: letter is an 8-bit ASCII character to be transferred
// Output: none

void UART_OutChar(unsigned char data)
{while((UART1_FR_R&UART_FR_TXFF) != 0);
UART1_DR_R = data;
}

void
uart_Init(void) {
	
		SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);


    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 38400,
        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}

//  red, yellow, green, light blue, blue, purple,  white,  dark

const long ColorWheel[3] = {0x02,0x04,0x00};
//interrupt handler
void GPIOPortF_Handler(void)
{

  unsigned char inColor;   // color value from other microcontroller

  // this microcontroller's color value

  while(1){
  long prevSW1 = 0;        // previous value of SW1
  long prevSW2 = 0;        // previous value of SW2
  unsigned char inColor;   // color value from other microcontroller
  unsigned char color = 0; // this microcontroller's color value
  
  while(1){
    unsigned long SW1,SW2;
		
    SW1 = GPIO_PORTF_DATA_R&0x10; // Read SW1

    if(SW1 == 0){    // falling of SW1?

      
			UART_OutChar(0x30);

    }
		
    prevSW1 = SW1; // current value of SW1

    SW2 = GPIO_PORTF_DATA_R&0x01; // Read SW2
    if(SW2 == 0){    // falling of SW1?

      
			UART_OutChar(0x31);

    }
    else if (SW1 == 0 && prevSW1 | SW2 == 0 && prevSW2) {   
			UART_OutChar(0x32);
		}
    prevSW2 = SW2; // current value of SW2
    
    inColor = UART_InCharNonBlocking();

    if(inColor){ // new data have come in from the UART??
      color = inColor&0x07;     // update this computer's color
    }

    GPIO_PORTF_DATA_R = ColorWheel[color];  // update LEDs
	
    }

     // update LEDs
			}
		}
   
   
int main(void)
{
		PLL_Init();
		//initialize the GPIO ports	
		PortFunctionInit();
	
		//initialize UART0
		uart_Init();
		
		//configure the GPIOF interrupt
		Interrupt_Init();
	
		IntMasterEnable();       		// globally enable interrupt
	 
   
	
    //
    // Loop forever.
    //
    while(1)
    {
		}
 
}


