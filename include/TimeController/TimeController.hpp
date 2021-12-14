#include <RtcDS3231.h>
#include <RtcDateTime.h>
#include <TimeLord.h>  //Biblioteca que indica posição do Sol. Pode ser removida e substituida por algo mais simples?
#include <Wire.h>      //Biblioteca para manipulação do protocolo I2C

RtcDateTime DateTime(__DATE__, __TIME__);  //Criação do objeto do tipo RTCDateTime iniciando com tempo do sistema
RtcDS3231<TwoWire> RTC(Wire);              //Criação do objeto do tipo DS3231

TimeLord Lord;

class TimeController {
   private:
    int _RTCAddress;

    int _Longitude = -43.2311486;  //Configurado para Rio de Janeiro
    int _Latitude = -22.8613427;
    int _Timezone = -3;

    double Day_Current_Time;
    double Day_Sun_Rise;
    double Day_Sun_Set;

   public:
    TimeController(int RTCAddress = 0x68) {
        _RTCAddress = RTCAddress;  //Sem uso - recebe endereço do RTC
    }

    void init() {
        RTC.Begin();                           //Inicialização do RTC DS3231
        RTC.SetDateTime(DateTime);             //Configurando valores iniciais do RTC DS3231
        Lord.TimeZone(_Timezone * 60);         //Envio de informações para TimeLord
        Lord.Position(_Latitude, _Longitude);  //
    }

    void CallRTC() {
        DateTime = RTC.GetDateTime();  // Atualiza o horário

#ifdef DEBUG_RTC  //Printa data e hora na tela
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
#endif
    }

    int sun_position() {
#ifndef TEST_SETPOINT_0

        byte today[] = {DateTime.Second(), DateTime.Minute(), DateTime.Hour(), DateTime.Day(), DateTime.Month(), DateTime.Year()};

        Day_Current_Time = (double)DateTime.Hour() * 3600 + (double)DateTime.Minute() * 60 + (double)DateTime.Second();  //Hora atual em segundos

        if (Lord.SunRise(today))
            Day_Sun_Rise = (double)today[tl_hour] * 3600 + (double)today[tl_minute] * 60;  //converte para segundos
        if (Lord.SunSet(today))
            Day_Sun_Set = (double)today[tl_hour] * 3600 + (double)today[tl_minute] * 60;  //converte para segundos

        return (map(Day_Current_Time, Day_Sun_Rise, Day_Sun_Set, 90.0, -90.0));  //Setpoint: conversão do horário atual em segundos para o equivalente em graus dados os extremos do ciclo solar variáveis e os extremos de angulação do sensor

#else

        return 0;

#endif
    }
};