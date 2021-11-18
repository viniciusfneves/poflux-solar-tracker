#include "PID.h"

#include <Arduino.h>

#define PWM_MINIMUM               -255    //Nao deixa mandar uma tensao acima de 4v para os motores
#define PWM_MAXIMUM               255  // 
#define INTEGRATIVE_MINIMUM       -200  //Nao deixa o integrativo aumentar muito em modulo
#define INTEGRATIVE_MAXIMUM       200 
#define KP            0.1      //Constante Proporcional 0.1
#define KI            0.25      //Constante Integrativa  0.2 
#define KD            0.016      //Constante Derivativa  0.016
#define KFE           1.0      //Constante de filtragem do erro (0: muito filtrado; 1: sem filtro)



int PID_Control :: PID(double error, bool debug, int threshold){
  

  dt = millis() - lastRun; 

  if (error > threshold)
  {
    P = KP * error;
    I = constrain (I + (KI * error * dt/1000.0), INTEGRATIVE_MINIMUM, INTEGRATIVE_MAXIMUM); 
    error = (KFE * error) + ((1- KFE) * lastError);                     //Filtra a variacao do erro para a derivada
    D = KD * ((error - lastError) / dt);
  }


  if(debug)
  {
    Serial.print("P = ");Serial.print(P); Serial.print("\t");
    Serial.print("I = ");Serial.print(I); Serial.print("\t");
    Serial.print("D = ");Serial.print(D); Serial.print("\t");
  }
  
  lastError = error;
  lastRun = millis();

  // --------------- Zerando I---------------------//
  
  if((error <= threshold) && (abs(I) >= 1))
  {
    if(!stabilization_test)
    {
      stabilization_time = millis();
      stabilization_test = true;
    }
    else if(stabilization_time + 1000 <= millis())
    {
      stabilization_time = 0;
      stabilization_test = false;
      I = 0;
    }
  }



  return (int) constrain (P + I + D, PWM_MINIMUM, PWM_MAXIMUM); 
     
}
