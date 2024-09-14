
#include "../inc/Comunicacion.h"
#include "../inc/UART.h"
#include "../inc/tm4c123gh6pm.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

long StartCritical(void);  
void EndCritical(long sr);

extern char modo;
extern uint16_t DistanciaEntreNivel;
extern uint16_t DistanciaEntreEspacio;
extern uint16_t DistanciaHomeAPrimerNivel;   
extern uint16_t DistanciaHomeAPrimerEspacio;   
extern uint16_t Xdestino[3];
extern int32_t Xactual[3];
extern int32_t Vactual[3];
extern bool SwitchOn[3];
extern uint16_t radioPolea[3];
extern bool NuevoDestino[3];
extern bool MotorDIR[3];
extern uint16_t DutyCycle[3];
extern bool EnableMotor[3];
extern int32_t Xorigen[3];

#define MOTOR_H 0
#define MOTOR_V 1
#define MOTOR_LONG 2

//Espera por el mensaje Modo y Coordenada que viene del Arduino.
//Termina esta funcion cuando ha recibido estos y ha actualizado
//los variables globales correspondientes.
void EsperaRxModoCoordenada(void){
	bool recibido = 0;
	while (!recibido){
		char msg = UART_InChar();
		//Si es un mensaje de Modo y Coordenada, procesar, sino ignorar.
		if (msg == 'd' || msg == 'r'){
			modo = msg;
			//Ahora esperar por los siguientes dos bytes que representa la coordenada.
			uint8_t Nivel = UART_InChar();
			uint8_t Espacio = UART_InChar();
			
			//Actualizar variables globales
			if (Espacio == 1) Xdestino[MOTOR_H] = DistanciaHomeAPrimerEspacio;
			else Xdestino[MOTOR_H] = DistanciaHomeAPrimerEspacio + (Espacio-1)*DistanciaEntreEspacio;
			
			if (Nivel == 1) Xdestino[MOTOR_V] = DistanciaHomeAPrimerNivel;
			else Xdestino[MOTOR_V] = DistanciaHomeAPrimerNivel + (Nivel-1)*DistanciaEntreNivel;
			
			//Ver si el destino asignado es diferente de la posicion actual. 
			//Si no es diferente, ignorar este mensaje y seguir esperando.
			if (abs(Xactual[MOTOR_H] - Xdestino[MOTOR_H]) > 5 || abs(Xactual[MOTOR_V] - Xdestino[MOTOR_V]) > 5){
				recibido = 1;
				if (abs(Xactual[MOTOR_H] - Xdestino[MOTOR_H]) > 5){
					NuevoDestino[MOTOR_H] = 1;
				}
				if (abs(Xactual[MOTOR_V] - Xdestino[MOTOR_V]) > 5){
					NuevoDestino[MOTOR_V] = 1;
				}				
			}
		}
	}
}

//Espera por mensaje de proceder que viene del Arduino. 
void EsperaRxProceder(void){
	bool recibido = 0;
	while (!recibido){
		char msg = UART_InChar();
		if (msg == 'p') recibido = 1;
	}
}

void PedirDatosMotoresForzado(void){
	UART_OutChar('x');
	UART_OutChar(LF);
	uint8_t MotoresFlags = 0;
	MotoresFlags = 1*0x01 + 1*0x02 +  1*0x04;
	UART_OutChar(MotoresFlags);
}

void PedirDatosMotores(void){
	UART_OutChar('x');
	UART_OutChar(LF);
	uint8_t MotoresFlags = 0;
	MotoresFlags = EnableMotor[MOTOR_H]*0x01 + EnableMotor[MOTOR_V]*0x02 +  EnableMotor[MOTOR_LONG]*0x04;
	UART_OutChar(MotoresFlags);
}

void PedirEstadoSwitch(void){
	UART_OutChar('m');
	UART_OutChar(LF);
}

void TxMotorDIR(void){
	UART_OutChar('D');
	UART_OutChar(LF);
	uint8_t msgMotorDIR = MotorDIR[0]*0x01 + MotorDIR[1]*0x02 + MotorDIR[2]*0x04;
	UART_OutChar(msgMotorDIR);
}

//Para enviar mensajes de DutyCycle
char DutyCycleMsgID[3] = {'h','v','l'};
void TxDutyCycle(uint8_t motor_id){
	UART_OutChar(DutyCycleMsgID[motor_id]);
	UART_OutChar(LF);
	UART_TxUINT16(DutyCycle[motor_id]);
}

void RxDatosMotoresForzado(void){
	
	//Esperar hasta recibir los datos de todos los motores, habilitados o no.
	uint8_t recibidoCount = 6;
	uint8_t cnt = 0;
	while (!(cnt == recibidoCount)){
		char msg = UART_InChar();
		if (msg == 'a' || msg == 'b' || msg == 'c' || msg == 'e' || msg == 'f' || msg == 'g') cnt ++;
		switch (msg){
			//16 bits de velocidad angular (resolucion = 0.035rad/s = 2grados/s) del motor horizontal.
			case 'a':
			{
				//Esperar 2 bytes de msg
				int16_t w_horizontal = UART_RxINT16();
				//Convertir a velocidad lineal mm/s
				Vactual[MOTOR_H] = ((radioPolea[MOTOR_H]*w_horizontal*35)/100000);  //radioPolea*(w*0.035)
				break;
			}
			//16 bits de velocidad lineal (resolucion = 1 mm/s) del actuador vertical.
			case 'b':
			{
				int16_t v_vertical = UART_RxINT16();
				Vactual[MOTOR_V] = v_vertical;
				break;
			}
			
			//16 bits de velocidad angular (resolucion = 0.035rad/s)del motor longitudinal.
			case 'c':
			{
				int16_t w_long = UART_RxINT16();
				Vactual[MOTOR_LONG] = ((radioPolea[MOTOR_LONG]*w_long*35)/100000); //radioPolea*(w*0.035)
				break;
			}
			//16 bits de posicion angular (resolucion = 0.035rad) del motor horizontal.
			case 'e':
			{
				int16_t theta_horizontal = UART_RxINT16();
				Xactual[MOTOR_H] = ((radioPolea[MOTOR_H]*theta_horizontal*35)/100000) - Xorigen[MOTOR_H]; //radioPolea*(w*0.035)
				break;
			}
			//16 bits de posicion lineal (resolucion = 1 mm)del actuador vertical.
			case 'f':
			{
				int16_t x_vertical = UART_RxINT16();
				Xactual[MOTOR_V] = x_vertical - Xorigen[MOTOR_V];
				break;
			}
			//16 bits de posicion angular (resolucion = 0.035rad) del motor longitudinal.
			case 'g':
			{
				int16_t theta_long = UART_RxINT16();
				Xactual[MOTOR_LONG] = ((radioPolea[MOTOR_LONG]*theta_long*35)/100000) - Xorigen[MOTOR_LONG]; //radioPolea*(w*0.035)
				break;
			}
		}
	}
}


void RxDatosMotores(void){
	
	//Esperar hasta recibir los datos de todos los motores habilitados.
	uint8_t recibidoCount = 2*(EnableMotor[1] + EnableMotor[0] + EnableMotor[2]);
	uint8_t cnt = 0;
	while (!(cnt == recibidoCount)){
		char msg = UART_InChar();
		if (msg == 'a' || msg == 'b' || msg == 'c' || msg == 'e' || msg == 'f' || msg == 'g') cnt ++;
		switch (msg){
			//16 bits de velocidad angular (resolucion = 0.035rad/s = 2grados/s) del motor horizontal.
			case 'a':
			{
				//Esperar 2 bytes de msg
				int16_t w_horizontal = UART_RxINT16();
				//Convertir a velocidad lineal mm/s
				Vactual[MOTOR_H] = ((radioPolea[MOTOR_H]*w_horizontal*35)/100000);  //radioPolea*(w*0.035)
				break;
			}
			//16 bits de velocidad lineal (resolucion = 1 mm/s) del actuador vertical.
			case 'b':
			{
				int16_t v_vertical = UART_RxINT16();
				Vactual[MOTOR_V] = v_vertical;
				break;
			}
			
			//16 bits de velocidad angular (resolucion = 0.035rad/s)del motor longitudinal.
			case 'c':
			{
				int16_t w_long = UART_RxINT16();
				Vactual[MOTOR_LONG] = ((radioPolea[MOTOR_LONG]*w_long*35)/100000); //radioPolea*(w*0.035)
				break;
			}
			//16 bits de posicion angular (resolucion = 0.035rad) del motor horizontal.
			case 'e':
			{
				int16_t theta_horizontal = UART_RxINT16();
				Xactual[MOTOR_H] = ((radioPolea[MOTOR_H]*theta_horizontal*35)/100000) - Xorigen[MOTOR_H]; //radioPolea*(w*0.035)
				break;
			}
			//16 bits de posicion lineal (resolucion = 1 mm)del actuador vertical.
			case 'f':
			{
				int16_t x_vertical = UART_RxINT16();
				Xactual[MOTOR_V] = x_vertical - Xorigen[MOTOR_V];
				break;
			}
			//16 bits de posicion angular (resolucion = 0.035rad) del motor longitudinal.
			case 'g':
			{
				int16_t theta_long = UART_RxINT16();
				Xactual[MOTOR_LONG] = ((radioPolea[MOTOR_LONG]*theta_long*35)/100000) - Xorigen[MOTOR_LONG]; //radioPolea*(w*0.035)
				break;
			}
		}
	}
}
		

void RxEstadoSwitch(void){
	bool recibido = 0;
	while (!recibido){
		char msg = UART_InChar();
		if (msg == 'm'){
			recibido = 1;
			//Estado de los tres switches: 'm' --> 8 bits 00000xxx
			uint8_t EstadoSwitch = UART_InChar();
			SwitchOn[MOTOR_H] = EstadoSwitch & 0x01;
			SwitchOn[MOTOR_V] = (EstadoSwitch & 0x02) >> 1;
			SwitchOn[MOTOR_LONG] = (EstadoSwitch & 0x04) >> 2;
		}
	}
}
	
		


