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

#define FiveMS 0x64
#define OneMS 0x14
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

enum eState{GoSouth = 0x00, WaitSouth = 0X01, GoWest = 0x02, WaitWest = 0X03, GoPedestrians = 0x04, WaitPedestriansY1 = 0X05, WaitPedestriansO1 = 0X06, WaitPedestriansY2 = 0X07, WaitPedestriansO2 = 0X08, stateMax = 0x09};
enum eInput{inputNone = 0x00, inputW = 0x01, inputS = 0x02, inputWS = 0x03, inputP = 0x04, inputPW = 0x05, inputPS = 0x06, inputPSW = 0x07, inputMAX = 0x08};

struct TState {
  void (*functionOut) (void); 
  unsigned long Time;
  enum eState Next[inputMAX];
};
typedef struct TState Fsm;

// FUNCTION PROTOTYPES: Each subroutine defined
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void PortB_Init(void);
void PortE_Init(void);
void PortF_Init(void);
void Port_Init(void);

void PedestriansOff(void);
void CarRed(void);
void CarOff(void);
void AllOff(void);
void WestYellow(void);
void WestGreen(void);
void SouthYellow(void);
void SouthGreen(void);
void PedestriansGreen(void);
void PedestriansRed(void);
void Delay100ms(unsigned long time);

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

void PedestriansOff(void){
	GPIO_PORTF_DATA_R &= ~(PIN(PEDGREEN)|PIN(PEDRED));
}

void PedestriansRed(void){
	GPIO_PORTF_DATA_R = PIN(PEDRED);
}

void AllOff(void){
	CarOff();
	PedestriansOff();
}

void CarOff(void){
	GPIO_PORTB_DATA_R &= ~(PIN(WESTRED)|PIN(WESTYELLOW)|PIN(WESTGREEN)|PIN(SOUTHRED)|PIN(SOUTHYELLOW)|PIN(SOUTHGREEN));
}

void CarRed(void){
	GPIO_PORTB_DATA_R = PIN(WESTRED)|PIN(SOUTHRED);
}


void WestYellow(void){
	GPIO_PORTB_DATA_R = PIN(WESTYELLOW)|PIN(SOUTHRED);
	PedestriansRed();
}

void WestGreen(void){
	GPIO_PORTB_DATA_R = PIN(WESTGREEN)|PIN(SOUTHRED);
	PedestriansRed();
}

void SouthYellow(void){
	GPIO_PORTB_DATA_R = PIN(SOUTHYELLOW)|PIN(WESTRED);
	PedestriansRed();
}

void SouthGreen(void){
	GPIO_PORTB_DATA_R = PIN(SOUTHGREEN)|PIN(WESTRED);
	PedestriansRed();
}

void PedestriansGreen(void){
	CarRed();
	GPIO_PORTF_DATA_R = PIN(PEDGREEN);
}

void Delay100ms(unsigned long time){
  unsigned long i;
  while(time > 0){
    i =  71933; // 0.05sec in simulation //1333333;  // this number means 100ms
    while(i > 0){
      i = i - 1;
    }
    time = time - 1; // decrements every 100 ms
  }
}


int main(void){ 
	enum eState currentState = GoSouth;
	Fsm fsm[stateMax] ={
		{SouthGreen, FiveMS, {GoSouth, WaitSouth, GoSouth, WaitSouth,  WaitSouth,  WaitSouth,  WaitSouth,  WaitSouth}},
		{SouthYellow, FiveMS, {GoWest, GoWest, GoWest, GoWest,  GoPedestrians,  GoWest,  GoPedestrians,  GoWest}},
		{WestGreen, FiveMS, {GoWest, GoWest, WaitWest, WaitWest,  WaitWest,  WaitWest,  WaitWest,  WaitWest}},
		{WestYellow, FiveMS, {GoPedestrians, GoPedestrians, GoSouth, GoSouth,  GoPedestrians,  GoPedestrians,  GoPedestrians,  GoPedestrians}},
		{PedestriansGreen, FiveMS, {GoPedestrians, WaitPedestriansY1, WaitPedestriansY1, WaitPedestriansY1,  GoPedestrians,  WaitPedestriansY1,  WaitPedestriansY1,  WaitPedestriansY1}},
		{PedestriansOff, OneMS, {WaitPedestriansO1, WaitPedestriansO1, WaitPedestriansO1, WaitPedestriansO1,  WaitPedestriansO1,  WaitPedestriansO1,  WaitPedestriansO1,  WaitPedestriansO1}},
		{PedestriansGreen, OneMS, {WaitPedestriansY2, WaitPedestriansY2, WaitPedestriansY2, WaitPedestriansY2,  WaitPedestriansY2,  WaitPedestriansY2,  WaitPedestriansY2,  WaitPedestriansY2}},
		{PedestriansOff, OneMS, {WaitPedestriansO2, WaitPedestriansO2, WaitPedestriansO2, WaitPedestriansO2,  WaitPedestriansO2,  WaitPedestriansO2,  WaitPedestriansO2,  WaitPedestriansO2}},
		{PedestriansGreen, OneMS, {GoSouth, GoWest, GoSouth, GoSouth,  GoSouth,  GoWest,  GoSouth,  GoSouth}},
	};
	enum eInput input;
	
  TExaS_Init(SW_PIN_PE210, LED_PIN_PB543210,ScopeOff); // activate grader and set system clock to 80 MHz
	Port_Init();

  
  EnableInterrupts();

  while(1){
		fsm[currentState].functionOut();
		Delay100ms(fsm[currentState].Time);
		input = (enum eInput)(GPIO_PORTE_DATA_R & (PIN(2)|PIN(1)|PIN(0)));
		currentState = fsm[currentState].Next[input];
  }
}
