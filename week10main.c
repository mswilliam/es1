// ***** 0. Documentation Section *****
// TableTrafficLight.c for Lab 10
// Runs on LM4F120/TM4C123
// Index implementation of a Moore finite state machine to operate a traffic light.  
// Daniel Valvano, Jonathan Valvano
// January 15, 2016

// east/west red light connected to PB5
// east/west yellow light connected to PB4
// east/west green light connected to PB3
// north/south facing red light connected to PB2
// north/south facing yellow light connected to PB1
// north/south facing green light connected to PB0
// pedestrian detector connected to PE2 (1=pedestrian present)
// north/south car detector connected to PE1 (1=car present)
// east/west car detector connected to PE0 (1=car present)
// "walk" light connected to PF3 (built-in green LED)
// "don't walk" light connected to PF1 (built-in red LED)

// ***** 1. Pre-processor Directives Section *****
#include "TExaS.h"
#include "tm4c123gh6pm.h"
#define WESTGREEN 0X03
#define WESTYELLOW 0X04
#define WESTRED 0X05
#define SOUTHGREEN 0X00
#define SOUTHYELLOW 0X01
#define SOUTHRED 0X02
#define PEDGREEN 0X03
#define PEDRED 0X01
#define PIN(x) (0x01 << (x))

// ***** 2. Global Declarations Section *****

// FUNCTION PROTOTYPES: Each subroutine defined
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void PortB_Init(void);
void PortE_Init(void);
void PortF_Init(void);
void Port_Init(void);
void WestRedOn(void);
void WestYellowOn(void);
void WestGreenOn(void);
void SouthRedOn(void);
void SouthYellowOn(void);
void SouthGreenOn(void);
void PedestriansRedOn(void);
void PedestriansGreenOn(void);
void WestOff(void);
void SouthOff(void);
void PedestriansOff(void);
void AllOff(void);
void WestRed(void);
void WestYellow(void);
void WestGreen(void);
void SouthRed(void);
void SouthYellow(void);
void SouthGreen(void);
void PedestriansRed(void);
void PedestriansGreen(void);

// ***** 3. Subroutines Section *****
void PortB_Init(void){
	volatile unsigned long delay;
  SYSCTL_RCGC2_R |= PIN(1);      // 1) B clock
  delay = SYSCTL_RCGC2_R;            // delay to allow clock to stabilize     
  GPIO_PORTB_AMSEL_R &= 0x00;        // 2) disable analog function
  GPIO_PORTB_PCTL_R &= 0x00000000;   // 3) GPIO clear bit PCTL  
  GPIO_PORTB_DIR_R |= (PIN(5)|PIN(4)|PIN(3)|PIN(2)|PIN(1)|PIN(0));          // 4.2) PB5-PB0 output  
  GPIO_PORTB_AFSEL_R &= 0x00;        // 5) no alternate function      
  GPIO_PORTB_DEN_R |= (PIN(5)|PIN(4)|PIN(3)|PIN(2)|PIN(1)|PIN(0));          // 7) enable digital pins PB5-PB0
}

void PortE_Init(void){
	volatile unsigned long delay;
  SYSCTL_RCGC2_R |= PIN(4);      // 1) E clock
  delay = SYSCTL_RCGC2_R;            // delay to allow clock to stabilize     
  GPIO_PORTE_AMSEL_R &= 0x00;        // 2) disable analog function
  GPIO_PORTE_PCTL_R &= 0x00000000;   // 3) GPIO clear bit PCTL  
  GPIO_PORTE_DIR_R &= ~(PIN(2)|PIN(1)|PIN(0));         // 4.1) PE2-PE0 input,  
  GPIO_PORTE_AFSEL_R &= 0x00;        // 5) no alternate function      
  GPIO_PORTE_DEN_R |= (PIN(2)|PIN(1)|PIN(0));          // 7) enable digital pins PE2-PE0
}

void PortF_Init(void){
	volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000020;      // 1) F clock
  delay = SYSCTL_RCGC2_R;            // delay to allow clock to stabilize     
  GPIO_PORTF_AMSEL_R &= 0x00;        // 2) disable analog function
  GPIO_PORTF_PCTL_R &= 0x00000000;   // 3) GPIO clear bit PCTL  
  GPIO_PORTF_DIR_R |= (PIN(3)|PIN(1));          // 4.2) PF3,1 output  
  GPIO_PORTF_AFSEL_R &= 0x00;        // 5) no alternate function      
  GPIO_PORTF_DEN_R |= (PIN(3)|PIN(1));          // 7) enable digital pins PF3,1
}

void Port_Init(void){
	PortB_Init();
	PortE_Init();
	PortF_Init();
}

void WestRedOn(void){
	GPIO_PORTB_DATA_R |= PIN(WESTRED);
}

void WestYellowOn(void){
	GPIO_PORTB_DATA_R |= PIN(WESTYELLOW);
}

void WestGreenOn(void){
	GPIO_PORTB_DATA_R |= PIN(WESTGREEN);
}

void SouthRedOn(void){
	GPIO_PORTB_DATA_R |= PIN(SOUTHRED);
}

void SouthYellowOn(void){
	GPIO_PORTB_DATA_R |= PIN(SOUTHYELLOW);
}

void SouthGreenOn(void){
	GPIO_PORTB_DATA_R |= PIN(SOUTHGREEN);
}


void PedestriansRedOn(void){
	GPIO_PORTF_DATA_R |= PIN(PEDRED);
}

void PedestriansGreenOn(void){
	GPIO_PORTF_DATA_R |= PIN(PEDGREEN);
}

void WestOff(void){
	GPIO_PORTB_DATA_R &= ~(PIN(WESTRED)|PIN(WESTYELLOW)|PIN(WESTGREEN));
}

void SouthOff(void){
	GPIO_PORTB_DATA_R &= ~(PIN(SOUTHRED)|PIN(SOUTHYELLOW)|PIN(SOUTHGREEN));
}

void PedestriansOff(void){
	GPIO_PORTF_DATA_R &= ~(PIN(PEDGREEN)|PIN(PEDRED));
}

void AllOff(void){
	WestOff();
	SouthOff();
	PedestriansOff();
}

void WestRed(void){
	AllOff();
	WestRedOn();
}

void WestYellow(void){
	AllOff();
	WestYellowOn();
}

void WestGreen(void){
	AllOff();
	WestGreenOn();
}

void SouthRed(void){
	AllOff();
	SouthRedOn();
}

void SouthYellow(void){
	AllOff();
	SouthYellowOn();
}

void SouthGreen(void){
	AllOff();
	SouthGreenOn();
}

void PedestriansRed(void){
	AllOff();
	PedestriansRedOn();
}

void PedestriansGreen(void){
	AllOff();
	PedestriansGreenOn();
}

int main(void){ 
  TExaS_Init(SW_PIN_PE210, LED_PIN_PB543210,ScopeOff); // activate grader and set system clock to 80 MHz
	Port_Init();

  
  EnableInterrupts();

  while(1){
     
  }
}
