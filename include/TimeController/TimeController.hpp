#pragma once

#include <RtcDS3231.h>
#include <RtcDateTime.h>
#include <TimeLord.h>  //Biblioteca que indica posição do Sol. Pode ser removida e substituida por algo mais simples?
#include <Wire.h>      //Biblioteca para manipulação do protocolo I2C

RtcDateTime dateTime(__DATE__, __TIME__);  //Criação do objeto do tipo RTCDateTime iniciando com tempo do sistema
RtcDS3231<TwoWire> rtc(Wire);              //Criação do objeto do tipo DS3231

TimeLord lord;

class TimeController {
   private:
    int _rtcAddress;

    double _longitude = -43.2311486;  //Configurado para Rio de Janeiro
    double _latitude = -22.8613427;
    int _timezone = -3;

    double _dayCurrentTime;
    double _daySunRise;
    double _daySunSet;

   public:
    TimeController(int RTCAddress = 0x68) {
        _rtcAddress = RTCAddress;
    }

    void init() {
        rtc.Begin();                           //Inicialização do RTC DS3231
        rtc.SetDateTime(dateTime);             //Configurando valores iniciais do RTC DS3231
        lord.TimeZone(_timezone * 60);         //Envio de informações para TimeLord
        lord.Position(_latitude, _longitude);  //
    }

    void callRTC() { dateTime = rtc.GetDateTime(); /*Atualiza o horário*/ }

    int sunPosition() {
        byte today[] = {dateTime.Second(), dateTime.Minute(), dateTime.Hour(), dateTime.Day(), dateTime.Month(), static_cast<byte>(dateTime.Year())};

        _dayCurrentTime = (double)dateTime.Hour() * 3600 + (double)dateTime.Minute() * 60 + (double)dateTime.Second();  //Hora atual em segundos

        if (lord.SunRise(today))
            _daySunRise = (double)today[tl_hour] * 3600 + (double)today[tl_minute] * 60;  //converte para segundos
        if (lord.SunSet(today))
            _daySunSet = (double)today[tl_hour] * 3600 + (double)today[tl_minute] * 60;  //converte para segundos

        return (map(_dayCurrentTime, _daySunRise, _daySunSet, 90.0, -90.0));  //Setpoint: conversão do horário atual em segundos para o equivalente em graus dados os extremos do ciclo solar variáveis e os extremos de angulação do sensor
    }
};

TimeController timeInfo(0x68);
