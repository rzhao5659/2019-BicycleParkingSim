#include "tm4c123gh6pm.h"
#include <stdbool.h>
#include "../inc/UART.h"
#include "../inc/SensorIR.h"

//PF4 es el SW1. (Logica negativo) 
//Esta simulara el estado del sensor IR, y sera mostrado por el LED PF1 del Tiva. 

#define LF   0x0A
extern bool SensorIROn;

void SensorIR_Init(void){
// Activamos la señal de reloj del puerto F
SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5)==0);

GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port F
GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0
	
//Activar SW1 PF4 con Edge triggered interrupt
GPIO_PORTF_DIR_R &= ~(0x10); // PF4 pin de entrada
GPIO_PORTF_AFSEL_R &= ~(0x10); // no usamos función alternativa
GPIO_PORTF_PUR_R |= 0x10; // activamos resistencia de pull-up en pin PF4
GPIO_PORTF_DEN_R |= 0x10; // PF4 pin digital
GPIO_PORTF_IS_R &= ~0x11; // edge-sensitive
GPIO_PORTF_IBE_R &= ~0x11; // not both edges
GPIO_PORTF_IEV_R &= ~0x11; // falling edge event
GPIO_PORTF_ICR_R = 0x11; // clear flag
GPIO_PORTF_IM_R |= 0x11; // arm interrupt
NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000;
NVIC_EN0_R = 0x40000000;
	
//Activar LED rojo PF1
GPIO_PORTF_DIR_R |= 0x02; // Configura el bit 1 del puerto F como salida
GPIO_PORTF_DR8R_R |=0x02; // se activa el driver de 8 mA en el pin 1.
GPIO_PORTF_DEN_R |=0x02; // se activa el pin 1, como salida digital.
GPIO_PORTF_DATA_R &= ~(0X02); // apagamos el led
}

void GPIOPortF_Handler(void) { 
  GPIO_PORTF_ICR_R = 0x10; // acknowledge flag
	SensorIROn ^= 0x1; 
	//Cambiar el estado del LED y Enviar por UART al arduino el estado del sensor IR
	if (SensorIROn){
		GPIO_PORTF_DATA_R |= 0x02;
		UART_OutChar('s');
		UART_OutChar(LF);  //Letra de terminacion. 
	}
	else {
		GPIO_PORTF_DATA_R &= ~0x02;
		UART_OutChar('n');
		UART_OutChar(LF); 
	}
}
