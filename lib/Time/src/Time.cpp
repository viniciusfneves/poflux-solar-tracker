
#include "Time.h"

#include <TimeLord.h>                //Biblioteca Timelord para o cálculo do SunRise e Sunset
#include <Arduino.h>                //Biblioteca padrão do arduino para definição de byte

TimeLord Rio;

void SunLight ::  Sun_Range (const double LONG,const double LAT,const int ZONE)
{
  
  Rio.TimeZone(ZONE*60);              //Função interna de armazenamento do Fuso
  Rio.Position(LAT,LONG);             //Função interna de armazenamento da posição
   
  }

double SunLight ::  Sun_Position(uint8_t  RTC_seconds, uint8_t  RTC_minute, uint8_t  RTC_hour, uint8_t  RTC_day, uint8_t  RTC_month, uint8_t  RTC_year)
{
  
  byte today[] = {RTC_seconds,RTC_minute,RTC_hour,RTC_day,RTC_month,RTC_year};

  Day_Current_Time = (double)RTC_hour*3600 + (double)RTC_minute*60 + (double)RTC_seconds;     //Hora atual em segundos
 
  
  if (Rio.SunRise(today))                                                                   //Insere o horário do nascer do sol no array
  {                                                  
  Day_Sun_Rise =  (double)today[tl_hour]*3600 + (double)today[tl_minute]*60;            //converte para segundos
  }
 
  if (Rio.SunSet(today))                                                                    //Insere o horário do pôr do sol no array
  {                                                   
  Day_Sun_Set = (double)today[tl_hour]*3600  + (double)today[tl_minute]*60;      //converte para segundos
  }
  
  //Cálculos em segundos//
  Day_Range_Sun_Seconds = (Day_Sun_Set - Day_Sun_Rise) ;               //Tempo em segundos de sol no dia
  Day_Range_Sun_Degrees = ((double)Day_Range_Sun_Seconds/86400.0)*360.0 ;   //Hora em segundos da metade do tempo de sol do dia 
  Day_Angle_Sun_End = 90 - (Day_Range_Sun_Degrees/2) ;               //Angulo de íncio do ciclo solar 
  Day_Angle_Sun_Begin = 90 + (Day_Range_Sun_Degrees/2) ;                 //Angulo de fim do ciclo solar
  
  Sun_Angle = map(Day_Current_Time,Day_Sun_Rise,Day_Sun_Set,90.0,-90.0);   //conversão do horário atual em segundos para o equivalente em graus dados os extremos do ciclo solar variáveis e os extremos de angulação do sensor
  
  return Sun_Angle;
  
  //Serial.print("Sun Position: ");
  //Serial.print(Sun_Angle);
  
  
 }
