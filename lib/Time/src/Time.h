#include <stdint.h>

#ifndef _Time_H
#define _Time_H


class SunLight
{

  public:
    double Day_Current_Time;
    double Day_Sun_Rise;
    double Day_Sun_Set;

    double Day_Range_Sun_Seconds;          //Tempo em segundos de sol no dia
    double Day_Range_Sun_Degrees;             //Hora em segundos da metade do tempo de sol do dia 
    double Day_Angle_Sun_Begin;               //Angulo de íncio do ciclo solar 
    double Day_Angle_Sun_End;                 //Angulo de término do ciclo solar 

    float Sun_Angle;
   
    void Sun_Range (const double  LONG,const double  LAT,const int  ZONE); 
    double Sun_Position(uint8_t RTC_seconds, uint8_t RTC_minute, uint8_t RTC_hour, uint8_t RTC_day, uint8_t RTC_month, uint8_t RTC_year);
    
};




#endif
