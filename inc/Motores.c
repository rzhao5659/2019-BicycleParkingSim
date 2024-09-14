
#include "../inc/SysTick.h"
#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include "../inc/UART.h"
#include "../inc/Motores.h"
#include "../inc/Timer0A.h"
#include "../inc/Comunicacion.h"
#include "../inc/DEBUG.h"

//Usar Timer0A en modo periodico con interrupciones para el PID.
//Usar Systick para los retardos/esperas.


long StartCritical(void);  
void EndCritical(long sr);

#define MOTOR_H 0
#define MOTOR_V 1
#define MOTOR_LONG 2
#define LF 0x0A

extern uint16_t PosicionFinalActuadorLongitudinal;
extern uint16_t Xdestino[3];
extern bool EnableMotor[3];
extern int32_t Xactual[3];
extern int32_t Vactual[3];
extern bool LlegoAlDestino[3];
extern uint32_t TiempoDeEsperaGripper;
extern bool GripperOn;
extern uint16_t amax[3];
extern uint16_t vmax[3];
extern bool NuevoDestino[3];
extern int32_t Xorigen[3];

uint16_t KPX[3] = {4,5,2};    
uint16_t KIV[3] = {400,200,150};   // resolucion = 1/10^4. 
uint16_t KPV[3] = {30,30,124};     // resolucion = 1/10^4. 
int32_t I[3] = {0,0,0};

//La aceleracion y la velocidad lineal que moveran los actuadores lineales al destino. 
//aMov < amax.  vMov < vmax.
int16_t aMov[3];
int16_t vMov[3];
//Tiempo de duracion del etapa de velocidad constante y aceleracion/desaceleracion constante 
int32_t Tvcte[3];  //ms
int32_t Tacte[3];

void PerfilMovimiento(int16_t d, uint8_t motor_id){  
	//d representa desplazamiento en mm.
	//Determinar signo de desplazamiento y corregir.
	int8_t signo = 1; 
	if (d < 0){
		d = -d;
		signo = -1;
	}
	//Iniciar la velocidad y la aceleracion de movimiento con sus valores maximos.
	int32_t v_mov = vmax[motor_id];
	int32_t a_mov = amax[motor_id];
	
	//Determinar el tiempo del etapa de aceleracion/desaceleracion constante
	Tacte[motor_id] = v_mov*1000/a_mov;
	
	//Determinar si la distancia requiere un perfil trapezoidal. 
	if (d > v_mov*Tacte[motor_id]/1000){  //d > 2*(vmax/2)*Tacte
		//Necesita una region con Vmax constante. Trapezoidal
		Tvcte[motor_id] = (d*1000/v_mov - Tacte[motor_id]);  //d = 2*(0.5*vmax)*Tacte + vmax*Tvcte
	}
	else{
		//Si no es necesario un perfil trapezoidal. 
		//Se busca un perfil que minimice el tiempo de viaje con un perfil triangular. 
		v_mov = sqrt(a_mov*d);
		Tacte[motor_id] = v_mov*1000/a_mov;
		Tvcte[motor_id] = 0;
	}
	
	//Estas seran la velocidad y la aceleracion que empleara el actuador correspondiente 
	//al motor_id para viajar esta cantidad de desplazamiento d. 
	vMov[motor_id] = v_mov * signo;
	aMov[motor_id] = a_mov * signo;
}



void AvanzarGripper(void){
	//Actualizar la posicion deseada
	Xdestino[MOTOR_LONG] = PosicionFinalActuadorLongitudinal;
	NuevoDestino[MOTOR_LONG] = 1;
	
	//Habilitar el control del motor.
	EnableMotor[MOTOR_LONG] = 1;
	
	//Habilitar envio de datos de motores de Simulink a Tiva: posiciones y velocidades
	//Esto servira para actualizar la posicion actual Xactual. 
	PedirDatosMotores();
	RxDatosMotores();
	
	//Generar un perfil de movimiento como referencia para el control PID.
	int16_t desplazamiento = Xdestino[MOTOR_LONG] - Xactual[MOTOR_LONG];
	PerfilMovimiento(desplazamiento, MOTOR_LONG);

	//Habilitar el Timer periodico del PID (control de posicion)
	EnablePID();
	
	//Esperar hasta llegar a la posicion deseada. 
	while (!LlegoAlDestino[MOTOR_LONG]);
	LlegoAlDestino[MOTOR_LONG] = 0;
	
	//Deshabilitar el motor cuando llega y deshabilitar el envio de sus salidas en Simulink.
	EnableMotor[MOTOR_LONG] = 0;
	DisablePID();
}



void RetrocederGripper(void){
	//Actualizar la posicion deseada
	Xdestino[MOTOR_LONG] = 0;
	NuevoDestino[MOTOR_LONG] = 1;
		
	//Habilitar el control del motor en el timer de PID.
	EnableMotor[MOTOR_LONG] = 1;
	
	//Habilitar envio de datos de Simulink a Tiva (Recibir salidas de los motores y switches)
	//Esto actualiza el Xactual antes de ejecutar el PID. 
	PedirDatosMotores();
	RxDatosMotores();
	
	//Generar un perfil de movimiento como referencia para el control PID.
	int16_t desplazamiento = Xdestino[MOTOR_LONG] - Xactual[MOTOR_LONG];
	PerfilMovimiento(desplazamiento, MOTOR_LONG);

	//Habilitar el Timer del PID para control de posicion.
	EnablePID();
	
	//Esperar hasta llegar a la posicion deseada. Durante la espera recibe datos de los motores(Simulink)
	while (!LlegoAlDestino[MOTOR_LONG]);
	LlegoAlDestino[MOTOR_LONG] = 0;
	
	//Deshabilitar el motor cuando llega y deshabilitar sus salidas (Simulink)
	EnableMotor[MOTOR_LONG] = 0;
	DisablePID();
}

void AbrirGripper(void){
	if (GripperOn) return;
	else{
		GripperOn = 1;
		//Enviar mensaje AbrirGripper a Simulink. 
		UART_OutChar('i');
		UART_OutChar(LF);
		
		//Esperar TiempoDeEsperaGripper ms para que el Gripper se abra.
		SysTick80_Wait10ms(TiempoDeEsperaGripper/10);
	}
}

void CerrarGripper(void){
	if (!GripperOn) return;
	else{
		GripperOn = 0;
		//Enviar mensaje CerrarGripper a Simulink. 
		UART_OutChar('j');
		UART_OutChar(LF);
		
		//Esperar TiempoDeEsperaGripper ms para que el Gripper se cierre.
		SysTick80_Wait10ms(TiempoDeEsperaGripper/10);
	}
}


void TrasladarAlDestino(void){
	//Solo habilitar el motor necesario.
	if (NuevoDestino[MOTOR_H] == 1) EnableMotor[MOTOR_H] = 1;
	if (NuevoDestino[MOTOR_V] == 1) EnableMotor[MOTOR_V] = 1;
	
	//Habilitar envio de datos de motores de Simulink a Tiva: posiciones y velocidades
	//Esto servira para actualizar la posicion actual Xactual. 
	PedirDatosMotores();
	RxDatosMotores();
	
	//El EsperarRxModoCoordenada garantiza que existe un destino valido para al menos uno de estos motores
	if (NuevoDestino[MOTOR_H] == 1){
		//Generar un perfil de movimiento como referencia para el control PID.
		int16_t desplazamiento = Xdestino[MOTOR_H] - Xactual[MOTOR_H];
		PerfilMovimiento(desplazamiento, MOTOR_H);
	}
	if (NuevoDestino[MOTOR_V] == 1){
		int16_t desplazamiento = Xdestino[MOTOR_V] - Xactual[MOTOR_V];
		PerfilMovimiento(desplazamiento, MOTOR_V);
	}
	
	//Habilitar el Timer periodico del PID (control de posicion)
	EnablePID();
	
	//Esperar hasta llegar a la posicion deseada.
	if (EnableMotor[MOTOR_V] == 1 && EnableMotor[MOTOR_H] == 1){
		while (!(LlegoAlDestino[MOTOR_V] && LlegoAlDestino[MOTOR_H]));
		LlegoAlDestino[MOTOR_V] = 0;
		LlegoAlDestino[MOTOR_H] = 0;
	}
	else if (EnableMotor[MOTOR_V] == 1 && EnableMotor[MOTOR_H] == 0){
		while (!(LlegoAlDestino[MOTOR_V]));
		LlegoAlDestino[MOTOR_V] = 0;
	}
	else{
		while (!(LlegoAlDestino[MOTOR_H]));
		LlegoAlDestino[MOTOR_H] = 0;
	}
	
	//Deshabilitar los motores cuando llegan.
	EnableMotor[MOTOR_V] = 0;
	EnableMotor[MOTOR_H] = 0;
	DisablePID();
}


void TrasladarAlOrigen(void){
	//Cambiar Xdestino al origen. Xdestino siempre es con respecto a Xorigen. 
	Xdestino[MOTOR_V] = 0;
	Xdestino[MOTOR_H] = 0;
	
	//Actualizar Xactual;
  PedirDatosMotoresForzado();
	RxDatosMotoresForzado();
	
	//Considerar nuevo destino solo si es la diferencia de distancia es mayor que 5mm
	if (abs(Xdestino[MOTOR_V] - Xactual[MOTOR_V]) < 5);
	else NuevoDestino[MOTOR_V] = 1;
		
	if (abs(Xdestino[MOTOR_H] - Xactual[MOTOR_H]) < 5);
	else NuevoDestino[MOTOR_H] = 1;
		
	TrasladarAlDestino();
}



int32_t LimitarNumero (int32_t nro, int32_t ll, int32_t ul){
	if (nro < ll) nro = ll;
	else if (nro > ul) nro = ul;
	return nro;
}

//Variables necesarios para determinar la posicion y velocidad de referencia segun el perfil de movimiento generado.
int16_t XInicial[3] = {0,0,0};
uint16_t Tacum[3];  //en ms.
extern uint16_t DutyCycle[3];
extern bool MotorDIR[3];

void PIDcontrol(void){
	uint8_t i;  //motor_id
	//Leer las posiciones y velocidades actuales de los motores 
	PedirDatosMotores();
	RxDatosMotores();
  for (i = 0; i < 3; i++){
		//Solo controlar el motor habilitado. 
		if (EnableMotor[i]){
			//Configuraciones si recibe un nuevo destino. 
			if(NuevoDestino[i]){
				NuevoDestino[i] = 0;
				XInicial[i] = Xactual[i];
				Tacum[i] = 0;
			}
			//Ver si el actuador llego al Xdestino. 
			//5mm y 5mm/s de tolerancia en posicion y velocidad respectivamente
			if (abs(Xactual[i] - Xdestino[i]) <= 3 && abs(Vactual[i]) <= 3){
				LlegoAlDestino[i] = 1;
				if (DutyCycle[i] != 0){
					DutyCycle[i] = 0;
					TxDutyCycle(i);
				}
				continue;
			}
			//Tiempo actual:
			Tacum[i] += 10;  //El periodo del timer de PID es 10ms
			
			//Feedforward: Posicion y Velocidad de referencias
			//Se multiplicara estas variables por 100 para mejorar la exactitud numerica.
			int32_t Xref = 0;  
			int32_t Vref = 0;
			
			//Calcular la posicion y la velocidad de referencia del tiempo actual 
			//segun el perfil de movimiento.
			//Si es perfil trapezoidal
			if (Tvcte[i] != 0){
				//Etapa de aceleracion constante
				if (Tacum[i] <= Tacte[i]){  
					Vref = (aMov[i]*Tacum[i])/10;
					Xref = 100*XInicial[i] + (5*Vref*Tacum[i])/10000;  // 0.5*Vref*(Tacum/1000)
				}
				//Etapa de velocidad constante
				else if(Tacum[i] <= (Tvcte[i] + Tacte[i])){ 
					Vref = vMov[i]*100;
					int32_t X1 = 100*XInicial[i] + ((5*aMov[i])/10)*((Tacte[i]*Tacte[i])/10000);
					int32_t deltaT = Tacum[i] - Tacte[i];
					Xref = X1 + (Vref*deltaT)/1000;
				}
				//Etapa de desaceleracion constante
				else{   
					int32_t deltaT = Tacum[i] - (Tacte[i]+Tvcte[i]);
					if (deltaT < Tacte[i]){
						int32_t X2 = 100*XInicial[i] + ((5*aMov[i])/10)*((Tacte[i]*Tacte[i])/10000) + vMov[i]*Tvcte[i]/10;
						Vref = 100*vMov[i] - aMov[i]*deltaT/10;
						Xref = X2 + vMov[i]*deltaT/10 - ((5*aMov[i])/10)*((deltaT*deltaT)/10000);
					}
					//Cuando el tiempo actual excede el tiempo de trayectoria.
					else{
						Vref = 0;
						Xref = Xdestino[i]*100;
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
						int32_t X1 = 100*XInicial[i] + ((5*aMov[i])/10)*((Tacte[i]*Tacte[i])/10000);
						Vref = 100*vMov[i] - aMov[i]*deltaT/10;
						Xref = X1 + vMov[i]*deltaT/10 - ((5*aMov[i])/10)*((deltaT*deltaT)/10000);
					}
					else{
						//Cuando el tiempo actual excede el tiempo de trayectoria.
						Vref = 0;
						Xref = Xdestino[i]*100;
					}					
				}
			}
			
			//Regresar a sus valores reales de Xref y Vref (en mm y mm/s)
			Xref = Xref/100;
			Vref = Vref/100;
			
			//Position loop: Controlador P
			int32_t deltaX = Xref - Xactual[i];
			int32_t vel_cmd = KPX[i]*deltaX + Vref;
			
			//Velocity Loop: Controlador PI
			int32_t deltaV = vel_cmd - Vactual[i];
			int32_t DC = KPV[i]*deltaV + I[i];
			
			//Forward diff. Por eso actualizar despues.
			I[i] = I[i] + (KIV[i]*deltaV)/100;  
			I[i] = LimitarNumero(I[i], -10000, 10000);
			
			//Obtener el signo del DC para determinar direccion del motor. 
			//Limitar valor DC a 0 a 10000
			//Enviar mensaje de MotorDIR[i] si se actualizo a un valor nuevo.
			if (DC >= 0){
				if (MotorDIR[i] != 1){
					MotorDIR[i] = 1;
					TxMotorDIR();
				}
				DC = LimitarNumero(DC, 0, 10000);
			}
			else{
				if (MotorDIR[i] != 0){
					MotorDIR[i] = 0;
					TxMotorDIR();
				}
				DC = LimitarNumero(-DC,0,10000);
			}
			
			if (DutyCycle[i] != (uint16_t)DC){
				DutyCycle[i] = (uint16_t)DC;
				TxDutyCycle(i);
			}
		}
		
		//Motores deshabilitados enviar DutyCycle = 0. 
		else{
			if (DutyCycle[i] == 0);  //No enviar otra vez el mismo comando.
			else{
				DutyCycle[i] = 0;
				TxDutyCycle(i);
			}
		}
	}
}



int32_t VInicial[3] = {0,0,0};
// Usar vMov como Vdestino.



//int32_t DEBUG[4][200];
int32_t cnt = 0;

void PIDcontrolVelocidad(void){
	uint8_t i;  //motor_id
	
	//Leer las velocidades actuales de los motores 
	PedirDatosMotores();
	RxDatosMotores();
	
  for (i = 0; i < 3; i++){
		//Solo controlar el motor habilitado. 
		if (EnableMotor[i]){
			//Ver si hay nuevo comando de velocidad
			if(NuevoDestino[i]){
				NuevoDestino[i] = 0;
				VInicial[i] = Vactual[i];
				Tacum[i] = 0;
			}

			//Ver si ya llego al vMov[i]. 0mm/s de tolerancia
			if (abs(Vactual[i] - vMov[i]) == 0){
				//Mantener el mismo DutyCycle.
				continue;
			}
			
			Tacum[i] += 10;  //El periodo del timer de PID es 10ms
			
			//Multiplicar por 100; (luego seran divididas) 
			int32_t Vref = 0;
			
			//Determinar la velocidad de referencia
			if (Tacum[i] >= abs(vMov[i])*1000/amax[i]) Vref = vMov[i]*100;   //Tacte = vmov/amov.
			else if ((vMov[i]-Vactual[i]) < 0) Vref = -amax[i]*Tacum[i]/10;
			else Vref = amax[i]*Tacum[i]/10;
			Vref = Vref/100;
					
			//Velocity Loop
			int32_t deltaV = Vref - Vactual[i];
			int32_t DC = KPV[i]*deltaV + I[i];
					
			//Forward diff. Por eso actualizar despues. 
			I[i] = I[i] + (KIV[i]*deltaV)/100;  
			I[i] = LimitarNumero(I[i], -10000, 10000);
			
			//DEBUG
//			if (i == MOTOR_H){
//				DEBUG[0][cnt] = Vref;
//				DEBUG[1][cnt] = deltaV;
//				DEBUG[2][cnt] = DC;
//				DEBUG[3][cnt++] = I[i];
//			}
				
			//Obtener el signo para determinar MotorDIR y Limitar valor DC a 0 a 10000
			//Enviar mensaje de MotorDIR[i] si se actualizo a un valor nuevo.
			if (DC >= 0){
				if (MotorDIR[i] != 1){
					MotorDIR[i] = 1;
					TxMotorDIR();
				}
				DC = LimitarNumero(DC, 0, 10000);
			}
			else{
				if (MotorDIR[i] != 0){
					MotorDIR[i] = 0;
					TxMotorDIR();
				}
				DC = LimitarNumero(-DC,0,10000);
			}
			
			if (DutyCycle[i] != (uint16_t)DC){
				DutyCycle[i] = (uint16_t)DC;
				TxDutyCycle(i);
			}
		}
		//Motores deshabilitados enviar DutyCycle = 0. 
		else{
			if (DutyCycle[i] == 0);  //No enviar otra vez el mismo comando.
			else{
				DutyCycle[i] = 0;
				TxDutyCycle(i);
			}
		}
	}
}

//Periodo de los Timers son 10ms originalmente pero
//por Simulink se tuvo que incrementar a 1s.
//pero recuerda que Simulink esta simulando a una velocidad mas lenta, tal que 1s es equivalente a 10ms.

void EnablePIDVelocidad(void){ 
	//Iniciar el Timer periodico con Callback function PIDControlVelocidad, periodo de 10ms con prioridad de interrupcion 1.
	Timer0A_Init(&PIDcontrolVelocidad, 80000000 ,1); 
}

void EnablePID(void){ 
	//Iniciar el Timer periodico con Callback function PIDControl
	//Periodo = 10ms y Prioridad de Interrupcion = 1.
	Timer0A_Init(&PIDcontrol, 80000000 ,1); 
}


void DisablePID(void){
	Timer0A_Stop();
}


extern int16_t VelocidadHomingRetroceso;
extern int16_t VelocidadHomingAvance;
extern bool SwitchOn[3];
extern int32_t Xorigen[3];
bool grabar = 0;
uint8_t DEBUG2[3][1000];

void ProcesoHoming(void){	
	vMov[0] = VelocidadHomingRetroceso;
	vMov[1] = VelocidadHomingRetroceso;
	vMov[2] = VelocidadHomingRetroceso;
	
	NuevoDestino[0] = 1;
	NuevoDestino[1] = 1;
	NuevoDestino[2] = 1;
	
	EnableMotor[0] = 1;
	EnableMotor[1] = 1;
	EnableMotor[2] = 1;

	EnablePIDVelocidad();
	
	//Homing seran dos fases:  
	//Fase 1 es retroceder con velocidad VelocidadHomingRetroceso que es relativamente rapido.
	//Fase 1 finaliza cuando detecta el switch.
	//Fase 2 es avanzar con velocidad VelocidadHomingAvance que es lento.
	//Fase 2 finaliza cuando deja de detectar el switch.
	
	bool Fase1HomingFinalizado[3] = {0,0,0};
	bool Fase2HomingFinalizado[3] = {0,0,0};
	uint8_t i;
	
	while (!(Fase2HomingFinalizado[0] && Fase2HomingFinalizado[1] && Fase2HomingFinalizado[2])){
		long sr = StartCritical();  //No permitir PID interrupt hasta terminar de recibir los mensajes de Switch.
		PedirEstadoSwitch();
		RxEstadoSwitch();
		EndCritical(sr);
		
		for (i = 0; i < 3; i++){
			//Fase1.
			if (SwitchOn[i] && Fase1HomingFinalizado[i] == 0){
				Fase1HomingFinalizado[i] = 1;			
				vMov[i] = VelocidadHomingAvance;
				NuevoDestino[i] = 1;
				
				if(i == 1){
					grabar = 1;
				}
			}
			
			//Fase2
			if (!(SwitchOn[i]) && Fase1HomingFinalizado[i] == 1 && Fase2HomingFinalizado[i] == 0){
				Fase2HomingFinalizado[i] = 1;
				EnableMotor[i] = 0;  //Detener el motor.
				PedirDatosMotoresForzado(); //Leer pos actual.
				RxDatosMotoresForzado();
				Xorigen[i] = Xactual[i] + Xorigen[i]; //Guardar pos actual como origen.
			}	
			
			
			if(i == 1 && grabar == 1){
				DEBUG2[0][cnt] = SwitchOn[i];
				DEBUG2[1][cnt] = Fase1HomingFinalizado[i];
				DEBUG2[2][cnt++] = Fase2HomingFinalizado[i];
			}
		}
		//Cada 5ms recibir estado de los Switches
		SysTick80_Wait10ms(50);
	}
	
	DisablePID();
}