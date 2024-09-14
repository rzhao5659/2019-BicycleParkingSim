#ifndef COMUNICACION_H
#define COMUNICACION_H
#include <stdint.h>
void EsperaRxModoCoordenada(void);
void EsperaRxProceder(void);
void RevisarRxSimulink(void);
void TxMotorDIR(void);
void TxDutyCycle(uint8_t motor_id);
void PedirDatosMotores(void);
void PedirDatosMotoresForzado(void);
void RxDatosMotores(void);
void RxDatosMotoresForzado(void);
void PedirEstadoSwitch(void);
void RxEstadoSwitch(void);
#endif
