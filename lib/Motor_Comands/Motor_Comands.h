#ifndef _MOTOR_COMANDS_H
#define _MOTOR_COMANDS_H

class Driver_Setup {
  
  private:

  int LPWM, RPWM, ENABLE;

  public:

  void CW(int pwm);
  void CCW(int pwm);
  void Stop(int pwm);
    
  };



#endif
