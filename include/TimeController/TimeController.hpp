#include <RtcDS3231.h>
#include <RtcDateTime.h>
#include <Wire.h>                           //Biblioteca para manipulação do protocolo I2C

RtcDateTime DateTime(__DATE__, __TIME__);  //Criação do objeto do tipo RTCDateTime iniciando com tempo do sistema
RtcDS3231<TwoWire> RTC(Wire);              //Criação do objeto do tipo DS3231

class TimeController
{
    private:
    int _RTCAddress;


    
    public:

    TimeController(int RTCAddress = 0x68){
        _RTCAddress = RTCAddress;
    }

    void CallRTC (){
        DateTime = RTC.GetDateTime();
    }

    void printDateTime (){

        CallRTC();

        Serial.print(DateTime.Day());
        Serial.print("/");
        Serial.print(DateTime.Month());
        Serial.print("/");
        Serial.print(DateTime.Year());
        Serial.print("  ");
        Serial.print(DateTime.Hour());
        Serial.print(":");
        Serial.print(DateTime.Minute());
        Serial.print(":");
        Serial.print(DateTime.Second());
        Serial.print("  ");
    }





};