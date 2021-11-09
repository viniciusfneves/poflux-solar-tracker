#include "PID.h"

#include <Arduino.h>

#define PWM_MINIMUM               -255    //Nao deixa mandar uma tensao acima de 4v para os motores
#define PWM_MAXIMUM               255  // 
#define INTEGRATIVE_MINIMUM       -200  //Nao deixa o integrativo aumentar muito em modulo
#define INTEGRATIVE_MAXIMUM       200 
#define KP            0.1      //Constante Proporcional 0.1
#define KI            0.2      //Constante Integrativa  0.2 
#define KD            0.016      //Constante Derivativa  0.016
#define KFE           1.0      //Constante de filtragem do erro (0: muito filtrado; 1: sem filtro)



int PID_Control :: PID(double error){
  
  dt = millis() - lastRun;

  P = KP * error;
  I = constrain (I + (KI * abs(error) * dt/1000.0), INTEGRATIVE_MINIMUM, INTEGRATIVE_MAXIMUM); 
  error = (KFE * error) + ((1- KFE) * lastError);                     //Filtra a variacao do erro para a derivada
  D = KD * ((error - lastError) / dt);
  
  lastError = error;
  lastRun = millis();

  return (int) constrain (P + I + D, PWM_MINIMUM, PWM_MAXIMUM); 
  
  
  
  }
