// StepperTestMain.c
// Runs on LM4F120/TM4C123
// Test the functions provided by Stepper.c,
// 
// Before connecting a real stepper motor, remember to put the
// proper amount of delay between each CW() or CCW() step.
// Daniel Valvano
// September 12, 2013
// Modified by Min HE

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015
   Example 4.1, Programs 4.4, 4.5, and 4.6
   Hardware circuit diagram Figure 4.27

 Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

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
void move360(unsigned char);

unsigned char direction = 0; // No movement = 0, Foward = 1, Backward = 2
unsigned char press = 0;
unsigned char pulse = 0;
unsigned char moveType = 0; // Forward = 1, Backward = 2, Left Turn = 3, Right Turn = 4, None = 0
unsigned int rotations = 0; // .18 deg rotations per function call, 4000 = 360 deg
unsigned char rotating = 0;
unsigned char rotated = 0;
unsigned char sensor = 0;
unsigned char initialReset = 0;


int main(void){
	unsigned int i=0;
	PortF_Init();
	PortA_Init();
  Stepper_Init(40000); // 10 ms for stepper, *50 = 500 ms for LED flash
	EnableInterrupts();
	//LIGHT = 0x04;
	
	
	
  while(1){
		if(pulse == 1){
			
			switch (moveType){
				case 1: // Move Forward
					pulse = 0;
					Stepper_CW(0);
					StepperR_CW();
					rotations += 1;
					//LIGHT ^= 0x04;
					if(direction == 1 && press == 1 && rotations >= 3000 && sensor == 1 && rotated == 1){LIGHT = 0x04; direction = 0; rotations = 0; rotated = 0;}
					if(direction == 1 && press == 1 && rotations >= 6000 && rotated == 0){LIGHT = 0x06; rotating = 1; rotations = 0;}
					
					if(press == 2 && rotations >= 6000 && rotated == 1){LIGHT = 0x08; direction = 0; moveType = 0; rotations = 0; rotated = 0;}
					break;
				
				case 2: // Move Backward
					pulse = 0;
					Stepper_CCW(0);
					StepperR_CCW();
					rotations += 1;
					if(direction == 2 && rotations >= 3000 && rotated == 0){rotating = 1; rotations = 0;}
					break;
				
				case 3:
					pulse = 0;
					Stepper_CW(0);
					StepperR_CCW();
					rotations +=1;
					if(rotations >= 1200){rotated = 1; rotating = 0; direction = 1; rotations = 0;}
					break;
				
				case 4:
					pulse = 0;
					Stepper_CCW(0);
					StepperR_CW();
					rotations += 1;
					if(rotations >= 1100){rotated = 1; rotating = 0; direction = 1; rotations = 0;}
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
	
	// Pressing SW1 moves car forward
	if(GPIO_PORTF_RIS_R & 0x10){direction = 1; press = 1;}
	
	// Pressing SW2 moves car backward
	if(GPIO_PORTF_RIS_R & 0x01){direction = 2; press = 2;}
	
	GPIO_PORTF_ICR_R = 0x11;

}

void SysTick_Handler(void){
	
	if(direction == 1 && rotating == 0)
		{
			moveType = 1; pulse = 1;
		}
		
	else if(direction == 1 && rotating == 1)
		{
			moveType = 4; pulse = 1;
		}
	
	else if (direction == 2 && rotating == 0)
		{
			moveType = 2; pulse = 1;
		}
	else if (direction == 2 && rotating == 1)
		{
			moveType = 3; pulse = 1;
		}
	else
	{
		moveType = 0; direction = 0; pulse = 1;
	}
	
}

void GPIOPortA_Handler(void){
	if(GPIO_PORTA_DATA_R & 0x80){sensor = 1; 
															if(rotated == 1){direction = 1;} else{direction = press;} 
															if(initialReset == 0 && rotated == 1){rotations = 0; initialReset = 1; LIGHT = 0x0C;}} // Falling edge
	else{sensor = 0; direction = 0;} // Rising edge
	GPIO_PORTA_ICR_R = 0x80; // acknowledge
 
}



