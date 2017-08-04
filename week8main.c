// ***** 0. Documentation Section *****
// SwitchLEDInterface.c for Lab 8
// Runs on LM4F120/TM4C123
// Use simple programming structures in C to toggle an LED
// while a button is pressed and turn the LED on when the
// button is released.  This lab requires external hardware
// to be wired to the LaunchPad using the prototyping board.
// January 15, 2016
//      Jon Valvano and Ramesh Yerraballi

// ***** 1. Pre-processor Directives Section *****
#include "TExaS.h"
#include "tm4c123gh6pm.h"

#define MASK_BIT_0 0x01
#define MASK_BIT_1 0x02
#define PORT_E 0x10
// ***** 2. Global Declarations Section *****

// FUNCTION PROTOTYPES: Each subroutine defined
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts

int IsSwitchPressed(void);
void ToggleLED(void);
void SetLED(void);
void ClearLED(void);
void InitGpio(void);
void Delay100ms(unsigned long time);
void InitGpio(void){
	//Turn on the clock for Port E
	volatile unsigned long delay;
  SYSCTL_RCGC2_R |= PORT_E;     
  delay = SYSCTL_RCGC2_R;           // allow time for clock to start
	//Clear the PE0 and PE1 bits in Port F AMSEL to disable analog
	GPIO_PORTE_AMSEL_R &= ~(MASK_BIT_0 | MASK_BIT_1);        
	// Clear the PE0 and PE1 bit fields in Port F PCTL to configure as GPIO
	GPIO_PORTE_PCTL_R &= ~(MASK_BIT_0 | MASK_BIT_1);   
	//Set the Port E direction register so PE0 is an input and PE1 is an output
	GPIO_PORTE_DIR_R &= ~MASK_BIT_0;          
	GPIO_PORTE_DIR_R |= MASK_BIT_1;
	//Clear the PE0 and PE1 bits in Port F AFSEL to disable alternate functions
  GPIO_PORTE_AFSEL_R &= ~(MASK_BIT_0 | MASK_BIT_1);        
	//Set the PE0 and PE1 bits in Port F DEN to enable digital
	GPIO_PORTE_DEN_R |= (MASK_BIT_0 | MASK_BIT_1);          
	//No need to Set the PE0 and PE1 bit in Port F PUR to activate an internal pullup resistor
		//GPIO_PORTE_PUR_R |= MASK_BIT_0;          
		//GPIO_PORTE_PUR_R |= MASK_BIT_1;          
	// allow changes to PE0 and PE1
	GPIO_PORTE_CR_R |= (MASK_BIT_0 | MASK_BIT_1);           
}
void Delay100ms(unsigned long time){
  unsigned long i;
  while(time > 0){
    i = 1333333;  // this number means 100ms
    while(i > 0){
      i = i - 1;
    }
    time = time - 1; // decrements every 100 ms
  }
}
void ToggleLED(void){
	GPIO_PORTE_DATA_R ^= MASK_BIT_1;
}
// Subroutine sets Ready high
// Inputs:  None
// Outputs: None
// Notes:   friendly means it does not affect other bits in the port
void SetLED(void){
	GPIO_PORTE_DATA_R |= MASK_BIT_1;
}


// Subroutine clears Ready low
// Inputs:  None
// Outputs: None
// Notes:   friendly means it does not affect other bits in the port
void ClearLED(void){
	GPIO_PORTE_DATA_R &= ~MASK_BIT_1;
}
int IsSwitchPressed(void){
	return ((GPIO_PORTE_DATA_R & MASK_BIT_0) == MASK_BIT_0);
}
// ***** 3. Subroutines Section *****

// PE0, PB0, or PA2 connected to positive logic momentary switch using 10k ohm pull down resistor
// PE1, PB1, or PA3 connected to positive logic LED through 470 ohm current limiting resistor
// To avoid damaging your hardware, ensure that your circuits match the schematic
// shown in Lab8_artist.sch (PCB Artist schematic file) or 
// Lab8_artist.pdf (compatible with many various readers like Adobe Acrobat).
int main(void){ 
//**********************************************************************
// The following version tests input on PE0 and output on PE1
//**********************************************************************
  TExaS_Init(SW_PIN_PE0, LED_PIN_PE1, ScopeOn);  // activate grader and set system clock to 80 MHz
  InitGpio();	
  EnableInterrupts();           // enable interrupts for the grader
	SetLED();
  while(1){
    Delay100ms(1);
		if(IsSwitchPressed() != 0){
			ToggleLED();
		}else{
			SetLED();
		}
  }
  
}
