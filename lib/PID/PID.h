
#ifndef _PID_H
#define _PID_H




class PID_Control   
{

  private:

    unsigned int dt =0;
    unsigned long lastRun = 0;
    int lastError = 0;
    float P=0, I=0, D=0;
        
  
  public:

    int PID (double error);
 
  };



#endif
