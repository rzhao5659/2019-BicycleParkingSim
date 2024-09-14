
#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include "../inc/UART.h"
#include "../inc/Motores.h"
#include "../inc/Timer0A.h"
#include "../inc/Comunicacion.h"
#include "../inc/PLL.h"
#include "../inc/DEBUG.h"

//Usar Timer0A en modo periodico con interrupciones para el PID.
//Usar Systick para los retardos/esperas.

#define MOTOR_H 0
#define MOTOR_V 1
#define MOTOR_LONG 2
#define LF 0x0A


extern bool NuevoDestino[3];
extern bool EnableMotor[3];
extern int32_t Xactual[3];
extern int32_t Vactual[3];
extern uint16_t Xdestino[3];;
uint8_t finalizado = 0;

extern uint16_t amax[3];
extern uint16_t vmax[3];
extern int16_t aMov[3];
extern int16_t vMov[3];
extern uint32_t Tvcte[3]; 
extern uint32_t Tacte[3];
extern int16_t XInicial[3];
extern uint16_t Tacum[3];  //en ms.

//Maximo tiempo de terminacion de trayectoria 10s
//1er indice es Xref y 2do indice es Vref
int16_t LogData[2][1000];
int cnt = 0;

//Para verificar que los Xref y Vref calculados son correctos.
//No calcula los comandos DutyCycle y MotorDIR. Solo tiene como salida Xref y Vref.
//Guardar Xref y Vref en cada paso en un arreglo para luego exportarlo por UART.
void PIDcontrolTEST(void){
	uint8_t i;  //motor_id
  for (i = 0; i < 3; i++){
		//Solo controlar el motor habilitado. 
		if (EnableMotor[i]){
			//Ver si hay nuevo Xdestino
			if(NuevoDestino[i]){
				NuevoDestino[i] = 0;
				XInicial[i] = Xactual[i];
				Tacum[i] = 0;
			}
			
			Tacum[i] += 10;  //El periodo del timer de PID es 10ms
			
			//Usar fixed point = 100; 
			int32_t Xref = 0;  
			int32_t Vref = 0;
			int debug = 0;
			if (cnt > 200){
				debug = 1;
			}
			
			//Determinar la posicion y la velocidad de referencia segun el perfil de movimiento.
			//Si es perfil trapezoidal
			if (Tvcte[i] != 0){
				if (Tacum[i] <= Tacte[i]){  //aceleracion
					Vref = (aMov[i]*Tacum[i])/10;
					Xref = 100*XInicial[i] + (5*Vref*Tacum[i])/10000;  // 0.5*Vref*(Tacum/1000)
				}
				else if(Tacum[i] <= (Tvcte[i] + Tacte[i])){ //Vcte
					Vref = vMov[i]*100;
					int X1 = 100*XInicial[i] + ((5*aMov[i])/10)*((Tacte[i]*Tacte[i])/10000);
					int deltaT = Tacum[i] - Tacte[i];
					Xref = X1 + Vref*deltaT/1000;
				}
				else{   //desaceleracion
					int deltaT = Tacum[i] - (Tacte[i]+Tvcte[i]);
					if (deltaT < Tacte[i]){
						int X2 = 100*XInicial[i] + ((5*aMov[i])/10)*((Tacte[i]*Tacte[i])/10000) + vMov[i]*Tvcte[i]/10;
						Vref = 100*vMov[i] - aMov[i]*deltaT/10;
						Xref = X2 + vMov[i]*deltaT/10 - ((5*aMov[i])/10)*((deltaT*deltaT)/10000);
					}
					else{
						Vref = 0;
						Xref = Xdestino[i]*100;
						finalizado = 1;
					}
				}
			}
			
			//Si es perfil triangular
			else{
				if (Tacum[i] <= Tacte[i]){  //aceleracion
					Vref = (aMov[i]*Tacum[i])/10;
					Xref = 100*XInicial[i] + (5*Vref*Tacum[i])/10000;  // 0.5*Vref*(Tacum/1000)
				}
				else{ //desaceleracion
					int deltaT = Tacum[i] - Tacte[i];
					if (deltaT < Tacte[i]){
						int64_t X1 = 100*XInicial[i] + ((5*aMov[i])/10)*((Tacte[i]*Tacte[i])/10000);
						Vref = 100*vMov[i] - aMov[i]*deltaT/10;
						Xref = X1 + vMov[i]*deltaT/10 - ((5*aMov[i])/10)*((deltaT*deltaT)/10000);
					}
					else{
						Vref = 0;
						Xref = Xdestino[i]*100;
						finalizado = 1;
					}					
				}
			}
			
			//Guardar datos de cada paso
			LogData[0][cnt] = Xref/100;    //quitarle lo que se le multiplico
			LogData[1][cnt++] = Vref/100;       //quitarle lo que se le multiplico
			}
		}
	}



void EnablePIDTEST(void){  //Timer0A periodico de 10ms con prioridad de interrupcion 1. Ejecuta la funcion PIDcontrol.
	Timer0A_Init(&PIDcontrolTEST, 800000 ,1);
}

void DisablePIDTEST(void){
	Timer0A_Stop();
}


int main4(void){
	PLL_Init(Bus80MHz);       // 80  MHz
  UART_Init();              // initialize UART
	
//	//Caso 1
//	NuevoDestino[2] = 1;
//	EnableMotor[2] = 1;
//	Xdestino[2] = 5500;
//	vmax[MOTOR_LONG] = 1500;
//	amax[MOTOR_LONG] = 1000;
//	PerfilMovimiento(5500, MOTOR_LONG);
//	EnablePIDTEST();
//	while(!finalizado);
//	DisablePIDTEST();
	
//	//Caso 2:  Usando una posicion inicial = 0.5m 
//	NuevoDestino[2] = 1;
//	EnableMotor[2] = 1;
//	Xactual[2] = 500;
//	Xdestino[2] = 1000;
//	vmax[MOTOR_LONG] = 1500;
//	amax[MOTOR_LONG] = 1000;
//	PerfilMovimiento(500, MOTOR_LONG);
//	EnablePIDTEST();
//	while(!finalizado);
//	DisablePIDTEST();
	
	//Caso 3: 
	NuevoDestino[2] = 1;
	EnableMotor[2] = 1;
	Xdestino[2] = 5000;
	vmax[MOTOR_LONG] = 3000;
	amax[MOTOR_LONG] = 500;
	PerfilMovimiento(5000, MOTOR_LONG);
	EnablePIDTEST();
	while(!finalizado);
	DisablePIDTEST();
	//csv
	for (int i = 0; i < cnt; i++){
		UART_OutUDec(LogData[0][i]);
		UART_OutChar(',');
		UART_OutUDec(LogData[1][i]);
		UART_TxEndLine();
	}
	
	return 1;	
}