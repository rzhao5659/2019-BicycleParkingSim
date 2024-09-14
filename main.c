#include <stdint.h>
#include <stdbool.h>
#include "../inc/PLL.h"
#include "../inc/UART.h"
#include "../inc/SysTick.h"
#include "../inc/Motores.h"
#include "../inc/SensorIR.h"
#include "../inc/Comunicacion.h"
#include "../inc/DEBUG.h"

//Notas:
//PID sera realizado en un Timer periodico de 10ms:  Enviar MotorDIR y DutyCycles. 

//Main realiza dos funciones separados:
//1. Control de sistema de actuacion para la traslacion. En esta etapa el Arduino no realiza ningun comunicacion con el Tiva.
//Tiva simplemente recibe por UART en el main por busy-wait los estados de switches, las velocidades y las posiciones de los motores.
//y ocurre interrupcion periodicamente una funcion de PID para enviar MotorDIR y DC. 
//2. Maquina de estados finitos cuando el usuario esta interactuando con el sistema de estacionamiento por interfaz y acciones con la bicicleta. 
// Aqui ocurre comunicacion con Arduino. 

#define CR   0x0D
#define LF   0x0A

//Al tener tres motores, se usara arreglo 1-D de tamanio 3 para muchos variables globales. 
//Estos son sus indices. 
#define MOTOR_H 0
#define MOTOR_V 1
#define MOTOR_LONG 2

//Maquina de Estados Finitos para el Tiva tiene 7 Estados:
#define INICIO 0
#define ESPERAR_PUERTA_ABIERTA 1
#define PREPARAR_DEPOSITO_USUARIO 2
#define INICIAR_DEPOSITO 3
#define INICIAR_RECOJO 4
#define PREPARAR_RECOJO_USUARIO 5
#define REGRESAR_INICIO 6
uint8_t estado_fsm = INICIO;

//Parametros del sistema en mm o mm/s o mm/s^2 a menos que se le indique lo contrario
//uint16_t PosicionFinalActuadorLongitudinal = 300;
uint16_t PosicionFinalActuadorLongitudinal = 1930;
uint16_t DistanciaEntreNivel = 1200;
uint16_t DistanciaEntreEspacio = 550;
uint16_t DistanciaHomeAPrimerNivel = 0;   
uint16_t DistanciaHomeAPrimerEspacio = 1025;   //1024.6
uint16_t radioPolea[3] = {2387,5000,1671};  //en 10^-5 m.  No se usa el de Vertical ya que el encoder es lineal.
uint32_t TiempoDeEsperaGripper = 80000; //en ms
uint16_t amax[3] = {1500,500,1000}; //La aceleracion lineal maxima (magnitud abs) de los actuadores.   
uint16_t vmax[3] = {500,1250,1500}; //La velocidad lineal maxima (magnitud abs)de los actuadores. 

//Variables Globales 
//Necesario para control de sistema de actuacion.
uint16_t Xdestino[3] = {0,0,0};  //mm
bool LlegoAlDestino[3] = {0,0,0};
bool EnableMotor[3] = {0,0,0};
bool NuevoDestino[3] = {0,0,0};
uint16_t DutyCycle[3] = {0,0,0}; //Comandos de velocidad (rango 0 a 10000) 
bool MotorDIR[3] = {1,1,1};
int32_t Xactual[3] = {0,0,0};
int32_t Vactual[3] = {0,0,0};

bool GripperOn = 0;

//Necesario para comunicacion entre Tiva y Arduino.
bool SensorIROn = 0;  
char modo = 0;

//Necesario para el proceso de Homing.  Mensaje es 'k'.
//Guarda velocidad en vMov[3].
bool ModoControlVelocidad = 0;
int16_t VelocidadHomingRetroceso = -100; // mm/s
int16_t VelocidadHomingAvance = 10; // mm/s
int32_t Xorigen[3] = {0,0,0};
bool SwitchOn[3] = {0,0,0};
//Un programa que pueda mostrar el algoritmo de homing seria algo que envie por UART la posicion actual que cree el uC en el simulink.

int main(void){
  PLL_Init(Bus80MHz); // Configurar la frecuencia del reloj a 80  MHz
  UART_Init();        // Inicializar puerto serial con el Arduino
	SensorIR_Init();		// Inicializar Puerto I/O para el boton que simulara el sensor IR.
	SysTick_Init();   	// Inicializar un Timer periodico para realizar esperas en acciones como AbrirGripper.
	//ProcesoHoming();		// Calibracion de posicion de origen.
	//Iniciar Maquina de Estados Finitas:
	while(1){
		switch (estado_fsm){
			case INICIO:
				//Sistema de estacionamiento en posicion inicial. 
				EsperaRxModoCoordenada();
				//Ir al sgte estado
				if (modo == 'd') estado_fsm = ESPERAR_PUERTA_ABIERTA;
				else if (modo == 'r') estado_fsm = INICIAR_RECOJO;
				break;
			
			case ESPERAR_PUERTA_ABIERTA:
				//Esperar por el mensaje proceder de Arduino que indica que la puerta esta abierta
				EsperaRxProceder();
				//Ir al sgte estado
				estado_fsm = PREPARAR_DEPOSITO_USUARIO;
				break;
			
			case PREPARAR_DEPOSITO_USUARIO:
				//Estas acciones permitira al usuario introducir la bicicleta.
				AvanzarGripper();
				AbrirGripper();
				//Enviar proceder al Arduino para avisar que el usuario puede introducir la bicicleta ahora. 
				UART_OutChar('p');
				UART_OutChar(LF);
				//Esperar por el mensaje proceder de Arduino que le confirme ya puede iniciar el proceso de deposito.
				EsperaRxProceder();
				//Ir al sgte estado
				estado_fsm = INICIAR_DEPOSITO;
				break;
			
			case INICIAR_DEPOSITO:
				//Regresar el actuador longitudinal a su posicion inicial.
				CerrarGripper();
				RetrocederGripper();
				//Enviar proceder al Arduino para que cierre la Puerta
				UART_OutChar('p');
				UART_OutChar(LF);
				//Iniciar la traslacion vertical y horizontal al Xdestino para el deposito.
				TrasladarAlDestino();
				//Dejar la bicicleta en su espacio de almacenamiento
				AvanzarGripper();
				AbrirGripper();
				RetrocederGripper();
				//Regresar a la posicion inicial
				TrasladarAlOrigen();
				//Enviar el mensaje ready al Arduino para indicar que el sistema de estacionamiento 
				//esta listo para el siguiente proceso.
				UART_OutChar('r');
				UART_OutChar(LF);
				//Ir al sgte estado
				estado_fsm = INICIO;
				break;
			
			case INICIAR_RECOJO:
				//Trasladar al Xdestino para el recojo
				TrasladarAlDestino();
				//Recoger la bicicleta de su espacio de almacenamiento (riel de soporte)
				AbrirGripper();
				AvanzarGripper();
				CerrarGripper();
				RetrocederGripper();
				//Regresar a la posicion inicial
				TrasladarAlOrigen();
				//Enviar proceder al Arduino para que abra la Puerta
				UART_OutChar('p');
				UART_OutChar(LF);
				//Esperar por el mensaje proceder del Arduino, que indica que la puerta esta abierta.
				EsperaRxProceder();
				//Ir al sgte estado
				estado_fsm = PREPARAR_RECOJO_USUARIO;
				break;
			
			case PREPARAR_RECOJO_USUARIO:
				//Acciones para que el usuario pueda recoger la bicicleta.
				AvanzarGripper();
				AbrirGripper();
				//Enviar el mensaje proceder al arduino para indicar al usuario que ya puede recoger su bicicleta.
				UART_OutChar('p');
				UART_OutChar(LF);
				//Esperar por el mensaje proceder del Arduino para saber que el usuario ya ha recogido su bicicleta.
				EsperaRxProceder();
				//Ir al sgte estado
				estado_fsm = REGRESAR_INICIO;
				break;
			
			case REGRESAR_INICIO:
				//Regresar el actuador longitudinal a su posicion inicial.
				RetrocederGripper();
				//Enviar el mensaje ready al Arduino para indicar que el sistema de estacionamiento 
				//esta listo para el siguiente proceso.
				UART_OutChar('r');
				UART_OutChar(LF);
				//Ir al sgte estado
				estado_fsm = INICIO;
				break;
		}
	}
	
	return 1;
}

//DEBUG/TEST CODE:

//Test para la simulacion del Sensor IR por switch y led.  [Verificado]
//int main(void){
//  PLL_Init(Bus80MHz);       // 80  MHz
//  UART_Init();              // initialize UART
//	SensorIR_Init();
//	while(1);
//	return 1;
//}

//Test para la simulacion del Systick con una espera correcto.
//Verificar que el LED rojo conmuta cada 2 segundos.
//[Verificado]
//int main(void){
//  PLL_Init(Bus80MHz);       // 80  MHz
//  UART_Init();              // initialize UART
//	SysTick_Init();
//	LED_Rojo_Init();
//	while(1){
//		ConmutarLedRojo();
//		SysTick80_Wait10ms(TiempoDeEsperaGripper/10);
//	}
//	return 1;
//}

//Test para el perfil de velocidad: Comparar con los resultados de Simulink. 
//Usar motor vertical.
//[Verificado]
//int main(void){
//  PLL_Init(Bus80MHz);       // 80  MHz
//  UART_Init();              // initialize UART para print
//	SysTick_Init();
//	
//	char msg_resultado[300];
//	extern uint16_t aMov[3];
//	extern uint16_t vMov[3];
//	extern uint32_t Tvcte[3];  
//	extern uint32_t Tacte[3];
//	
//	//Caso 1: 
//	vmax[MOTOR_V] = 1500;
//	amax[MOTOR_V] = 1000;
//	UART_OutString("Caso 1: vmax 1.5m/s amax 1m/s^2 d = 5500");
//	UART_TxEndLine();
//	PerfilMovimiento(5500, MOTOR_V);
//	snprintf(msg_resultado, 300, "vmov %d mm/s amov %d mm/s^2 Tvcte %d ms Tacte %d ms", 
//					 vMov[MOTOR_V], aMov[MOTOR_V], Tvcte[MOTOR_V], Tacte [MOTOR_V]);
//	UART_OutString(msg_resultado);
//	UART_TxEndLine();

//	//Caso 2: 
//	vmax[MOTOR_V] = 1500;
//	amax[MOTOR_V] = 1000;
//	UART_OutString("Caso 2: vmax 1.5m/s amax 1m/s^2 d = 500");
//	UART_TxEndLine();
//	PerfilMovimiento(500, MOTOR_V);
//	snprintf(msg_resultado, 300, "vmov %d mm/s amov %d mm/s^2 Tvcte %d ms Tacte %d ms", 
//					 vMov[MOTOR_V], aMov[MOTOR_V], Tvcte[MOTOR_V], Tacte [MOTOR_V]);
//	UART_OutString(msg_resultado);
//	UART_TxEndLine();
//	
//	
//	//Caso 3: 
//	vmax[MOTOR_V] = 3000;
//	amax[MOTOR_V] = 500;
//	UART_OutString("Caso 3: vmax 3m/s amax 0.5m/s^2 d = 5000");
//	UART_TxEndLine();
//	PerfilMovimiento(5000, MOTOR_V);
//	snprintf(msg_resultado, 300, "vmov %d mm/s amov %d mm/s^2 Tvcte %d ms Tacte %d ms", 
//					 vMov[MOTOR_V], aMov[MOTOR_V], Tvcte[MOTOR_V], Tacte [MOTOR_V]);
//	UART_OutString(msg_resultado);
//	UART_TxEndLine();
//		
//	//Caso 4: 
//	vmax[MOTOR_V] = 500;
//	amax[MOTOR_V] = 5000;
//	UART_OutString("Caso 4: vmax 0.5m/s amax 5m/s^2 d = 50");
//	UART_TxEndLine();
//	PerfilMovimiento(50, MOTOR_V);
//	snprintf(msg_resultado, 300, "vmov %d mm/s amov %d mm/s^2 Tvcte %d ms Tacte %d ms", 
//					 vMov[MOTOR_V], aMov[MOTOR_V], Tvcte[MOTOR_V], Tacte [MOTOR_V]);
//	UART_OutString(msg_resultado);
//	UART_TxEndLine();
//	
//	while(1);
//	return 1;
//}



//TEST3:  Probar si funciona el avanzargripper() y retrocedergripper().  [VERIFICADO]
//int main(void){
//	
//  PLL_Init(Bus80MHz);       // 80  MHz
//  UART_Init();              // initialize UART
//	SysTick_Init();
//	
//	while(1){
//		char letra = UART_InChar();
//		if (letra == 'a'){
//			AvanzarGripper();
//		}
//		else if (letra == 'b'){
//			RetrocederGripper();
//		}
//	}
//	
//	return 1;
//}


//TEST5:  Probar si el PID funciona para los motores horizontal y vertical.
//int main3(void){
//	
//  PLL_Init(Bus80MHz);       // 80  MHz
//  UART_Init();              // initialize UART
//	SysTick_Init();
//	LED_Rojo_Init();
//	
//	NuevoDestino[1] = 1;
//	NuevoDestino[0] = 1;
//	Xdestino[1] = 1200;
//	Xdestino[0] = 2000;

//	while(1){
//		char letra = UART_InChar();
//		if (letra == 'a'){
//			TrasladarAlDestino();
//			TrasladarAlOrigen();
//			NuevoDestino[1] = 1;
//			NuevoDestino[0] = 1;
//			Xdestino[1] = 500;
//			Xdestino[0] = 500;
//			TrasladarAlDestino();
//			TrasladarAlOrigen();
//			ConmutarLedRojo();
//		}
//		else if (letra == 'b'){
//			AvanzarGripper();
//		}
//	}
//	
//	return 1;
//}

//Test code para ProcesoHoming
void TxOrigen(void){
	UART_OutChar('q');
	UART_OutChar(LF);
	UART_TxINT16(Xorigen[0]);
	UART_TxINT16(Xorigen[1]);
	UART_TxINT16(Xorigen[2]);
}

//int main(void){
//  PLL_Init(Bus80MHz); // Configurar la frecuencia del reloj a 80  MHz
//  UART_Init();        // Inicializar puerto serial con el Arduino
//	SensorIR_Init();		// Inicializar Puerto I/O para el boton que simulara el sensor IR.
//	SysTick_Init();   	// Inicializar un Timer periodico para realizar esperas en acciones como AbrirGripper.
//	LED_Rojo_Init();
//	
//	while(1){
//		PedirEstadoSwitch();
//		RxEstadoSwitch();
//		//Cada 2.5ms recibir estado de los Switches
//		SysTick80_Wait10ms(25);
//	}

//	while(1);
//	return 1;
//}


//Main para prueba del proceso Homing:
//int main(void){
//  PLL_Init(Bus80MHz); // Configurar la frecuencia del reloj a 80  MHz
//  UART_Init();        // Inicializar puerto serial con el Arduino
//	SensorIR_Init();		// Inicializar Puerto I/O para el boton que simulara el sensor IR.
//	SysTick_Init();   	// Inicializar un Timer periodico para realizar esperas en acciones como AbrirGripper.
//	LED_Rojo_Init();
//	
//	//Iniciar Xorigen con valores diferentes de 0:  
//	//Colocar el origen del actuador vertical y horizontal al nivel 2 y espacio 1 respectivamente. 
//	Xorigen[MOTOR_H] = 1025;
//	Xorigen[MOTOR_V] = 1200;
//	
//	//Trasladar al origen.
//	TxOrigen();
//	TrasladarAlOrigen();
//	
//	//Hacer proceso de Homing
//	ProcesoHoming();
//	
//	//Al cabo de Homing, Xorigen debe ser igual a la posicion donde se encuentran los interruptores. 
//	//(Simulink establece las posiciones de los interruptores de carrera).
//	TxOrigen();
//	
//	//Comprobar usando traslacion a nivel 2 y espacio 1.
//	EsperaRxModoCoordenada();
//	TrasladarAlDestino();
//	
//	while(1);
//	return 1;
//}


//int main(void){
//  PLL_Init(Bus80MHz); // Configurar la frecuencia del reloj a 80  MHz
//  UART_Init();        // Inicializar puerto serial con el Arduino
//	SensorIR_Init();		// Inicializar Puerto I/O para el boton que simulara el sensor IR.
//	SysTick_Init();   	// Inicializar un Timer periodico para realizar esperas en acciones como AbrirGripper.
//	

//	//Iniciar Xorigen con valores diferentes de 0:  
//	//Colocar el origen del actuador vertical y horizontal al nivel 2 y espacio 1 respectivamente. 
//	Xorigen[MOTOR_H] = 100;
//	Xorigen[MOTOR_V] = 100;

//	//Trasladar al origen.
//	TxOrigen();
//	TrasladarAlOrigen();
//	
//	//Hacer proceso de Homing
//	ProcesoHoming();
//	
//	//Al cabo de Homing, Xorigen debe ser la posicion donde se encuentran los interruptores. 
//	//(Simulink establece las posiciones de los interruptores de carrera.
//	TxOrigen();
//	
//	NuevoDestino[1] = 1;
//	NuevoDestino[0] = 1;
//	Xdestino[1] = 1200;
//	Xdestino[0] = 1025;

//	TrasladarAlDestino();
//		
//	while(1);
//	return 1;
//}


//TEST PID vel
//extern int16_t vMov[3];
//int main(void){
//  PLL_Init(Bus80MHz); // Configurar la frecuencia del reloj a 80  MHz
//  UART_Init();        // Inicializar puerto serial con el Arduino
//	SensorIR_Init();		// Inicializar Puerto I/O para el boton que simulara el sensor IR.
//	SysTick_Init();   	// Inicializar un Timer periodico para realizar esperas en acciones como AbrirGripper.
//	LED_Rojo_Init();
//	
//	vMov[0] = VelocidadHoming;
//	vMov[1] = VelocidadHoming;
//	vMov[2] = VelocidadHoming;
//	
//	NuevoDestino[0] = 1;
//	NuevoDestino[1] = 1;
//	NuevoDestino[2] = 1;
//	
//	EnableMotor[0] = 1;
//	EnableMotor[1] = 1;
//	EnableMotor[2] = 1;

//	EnablePIDVelocidad();	
//	
//	while(1);
//	
//	return 1;
//}
