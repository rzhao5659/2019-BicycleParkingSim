#include "tm4c123gh6pm.h"
#include <stdint.h>

void LED_Rojo_Init(void){
// Activamos la señal de reloj del puerto F
SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5)==0);
	
//Activar LED rojo PF1
GPIO_PORTF_DIR_R |= 0x02; // Configura el bit 1 del puerto F como salida
GPIO_PORTF_DR8R_R |=0x02; // se activa el driver de 8 mA en el pin 1.
GPIO_PORTF_DEN_R |=0x02; // se activa el pin 1, como salida digital.
GPIO_PORTF_DATA_R &= ~(0X02); // apagamos el led
}

void ConmutarLedRojo(void){
	GPIO_PORTF_DATA_R ^= (0X02);
}
