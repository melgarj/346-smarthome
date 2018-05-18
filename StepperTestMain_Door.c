// StepperTestMain.c
// Runs on LM4F120/TM4C123
// Test the functions provided by Stepper.c,
// 
// Before connecting a real stepper motor, remember to put the
// proper amount of delay between each CW() or CCW() step.
// Daniel Valvano
// September 12, 2013
// Modified by Min HE


// PD3 connected to driver for stepper motor coil A
// PD2 connected to driver for stepper motor coil A'
// PD1 connected to driver for stepper motor coil B
// PD0 connected to driver for stepper motor coil B'
#define LIGHT										(*((volatile unsigned long *)0x40025038))
#include <stdint.h>
#include "stepper.h"
#include "tm4c123gh6pm.h"
#define T1ms 16000    // assumes using 16 MHz PIOSC (default setting for clock source)

void PortF_Init(void);
void PortA_Init(void);
void EnableInterrupts(void);

unsigned char presence = 0; // init at closed state
unsigned char doorOpen = 0; // activates when door is fully opened
unsigned int ledCounter = 0;
unsigned int rotations = 0;
unsigned char toggle = 0;
unsigned char direction = 0;

// rotations = 3500, 0x02 = red, 0x08 = green, 0x04 = blue

int main(void){
	unsigned int i=0;
	PortF_Init();
	PortA_Init();
  Stepper_Init(40000); // 10 ms for stepper, *50 = 500 ms for LED flash
	EnableInterrupts();
	LIGHT = 0x08;
	
  while(1){
		// turn clockwise 180 degrees
		// Conditions: Door is closed, trigger is active, sensor is present/aligns with trigger
		if(toggle == 1){
			switch(direction){
				case 1:
					toggle = 0;
					Stepper_CCW(0);
					ledCounter += 1;
					rotations += 1;
					if(rotations >= 3500 && doorOpen == 0){
						LIGHT = 0x04;
						doorOpen = 1;
						rotations = 0;
						ledCounter = 0;
					}
					break;
				
				case 2:
					toggle = 0;
					Stepper_CW(0);
					ledCounter += 1;
					rotations += 1;
					if(rotations >= 3500 && doorOpen == 1){
						LIGHT = 0x08;
						doorOpen = 0;
						rotations = 0;
						ledCounter = 0;
					}
					break;
					
					
				default:
					if(doorOpen == 0){LIGHT = 0x04;}
					else{LIGHT = 0x08;}
					break;
			}
		}		
  }
}


void PortF_Init(void){volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000020; // (a) activate clock for port F
	delay = SYSCTL_RCGC2_R;
	GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock PortF PF0  // Unlock at beginning, broke code 
	GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0 
  //GPIO_PORTF_DIR_R &= ~0x11;    // (c) make PF4 and PF0 in (built-in button)
	GPIO_PORTF_DIR_R = 0x0E;          // 5)PF3,PF2,PF1 output   
  GPIO_PORTF_AFSEL_R &= ~0x1F;  //     disable alt funct on PF4
  GPIO_PORTF_DEN_R |= 0x1F;     //     enable digital I/O on PF4  
  GPIO_PORTF_PCTL_R &= ~0x000FFFFF; // configure PF4 as GPIO
  GPIO_PORTF_AMSEL_R = 0;       //     disable analog functionality on PF
  GPIO_PORTF_PUR_R |= 0x11;     //     enable weak pull-up on PF4
  GPIO_PORTF_IS_R &= ~0x11;     // (d) PF4 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x11;    //     PF4 is not both edges
  GPIO_PORTF_IEV_R &= ~0x11;    //     PF4 falling edge event
  GPIO_PORTF_ICR_R = 0x11;      // (e) clear flag4
  GPIO_PORTF_IM_R |= 0x11;      // (f) arm interrupt on PF4
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // (g) priority 5
  NVIC_EN0_R |= 0x40000000;      // (h) enable interrupt 30 in NVIC
}

void PortA_Init(void){
	volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x00000001;
	delay = SYSCTL_RCGC2_R;
	GPIO_PORTA_DIR_R &= ~0x80; // Input, PA7
	GPIO_PORTA_AFSEL_R &= ~0xFF;
	GPIO_PORTA_DEN_R |= 0x80;
	GPIO_PORTA_PCTL_R &= ~0xF0000000;
	GPIO_PORTA_AMSEL_R = 0;
	GPIO_PORTA_IS_R &= ~0x80;
	GPIO_PORTA_IBE_R |= 0x80; // Both edges
	GPIO_PORTA_ICR_R = 0x80;  
	GPIO_PORTA_IM_R |= 0x80;
	NVIC_PRI0_R = (NVIC_PRI0_R&0xFFFFFF00) | 0x00000080; //priority 4
	NVIC_EN0_R |= 0x00000001;
	
}	

void GPIOPortF_Handler(void){
	if(GPIO_PORTF_RIS_R & 0x10){
		if(doorOpen == 0){
			presence = 1;
		}
		else{
			presence = 0;
		}
		LIGHT = 0x02;

	}
	GPIO_PORTF_ICR_R = 0x11;

}

void SysTick_Handler(void){
	if(doorOpen == 0 && presence == 1){
		direction = 1; // up
		toggle = 1;
	}
	
	else if(doorOpen == 1 && presence == 0){
		direction = 2; //down
		toggle = 1;
	}
	
	else
	{
		direction = 0;
	}
	
	if(ledCounter >= 50){LIGHT ^= 0x02; ledCounter = 0;}
	
}

void GPIOPortA_Handler(void){
  if(GPIO_PORTA_DATA_R == 0x00 && doorOpen == 0){presence = 1; LIGHT = 0x02;}
  if(GPIO_PORTA_DATA_R == 0x80 && doorOpen == 1){presence = 0; LIGHT = 0x02;  }
  GPIO_PORTA_ICR_R = 0x80; // acknowledge
	
}

