#include "Motor_Comands.h"
#include <Arduino.h>

void Driver_Setup :: CW(int pwm) {
  
  analogWrite(ENABLE, pwm); //0-255
  digitalWrite(LPWM, HIGH);
  digitalWrite(RPWM, LOW);
 }

void Driver_Setup :: CCW(int pwm) {
  
  analogWrite(ENABLE, pwm); //0-255
  digitalWrite(LPWM, LOW);
  digitalWrite(RPWM, HIGH);
 }

void Driver_Setup :: Stop(int pwm) {
  
  analogWrite(ENABLE, pwm); //0-255
  digitalWrite(LPWM, LOW);
  digitalWrite(RPWM, LOW);
 }
