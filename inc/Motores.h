#ifndef MOTORES_H
#define MOTORES_H
#include <stdint.h>
void EnablePID(void);
void DisablePID(void);
void AvanzarGripper(void);
void TrasladarAlOrigen(void);
void TrasladarAlDestino(void);
void CerrarGripper(void);
void AbrirGripper(void);
void RetrocederGripper(void);
void PerfilMovimiento(int16_t d, uint8_t motor_id);
void ProcesoHoming(void);
void EnablePIDVelocidad(void);
#endif