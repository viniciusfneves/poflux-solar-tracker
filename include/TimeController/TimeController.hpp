#pragma once

#include <RtcDS3231.h>
#include <RtcDateTime.h>
#include <TimeLord.h>  //Biblioteca que indica posição do Sol. Pode ser removida e substituida por algo mais simples?
#include <Wire.h>      //Biblioteca para manipulação do protocolo I2C

#include <configurations/configurations.hpp>

// Criação do objeto do tipo RTCDateTime iniciando com tempo do sistema
RtcDateTime dateTime(__DATE__, __TIME__);
// Criação do objeto do tipo DS3231
RtcDS3231<TwoWire> rtc(Wire);
SemaphoreHandle_t  RTCSemaphore = xSemaphoreCreateMutex();

TimeLord lord;

class TimeController {
   private:
    int _rtcAddress;

    double _longitude = -43.2311486;  // Configurado para Rio de Janeiro
    double _latitude  = -22.8613427;
    int    _timezone  = -3;

    double _dayCurrentTime;
    double _daySunRise;
    double _daySunSet;

    int     _ciclePosition   = 0;
    bool    _cicleIncrement  = true;
    int64_t _lastCicleUpdate = 0;
    uint8_t _cicleLimit      = 60;

   public:
    TimeController(int RTCAddress = 0x68) { _rtcAddress = RTCAddress; }

    void init() {
        rtc.Begin();                    // Inicialização do RTC DS3231
        lord.TimeZone(_timezone * 60);  // Envio de informações para TimeLord
        lord.Position(_latitude, _longitude);
    }

    int getCicleSetpoint() { return _ciclePosition; }

    int timezone() { return _timezone; };

    void callRTC() {
        RtcDateTime rtcData = rtc.GetDateTime();
        if (rtcData.IsValid()) {
            xSemaphoreTake(RTCSemaphore, portMAX_DELAY);
            dateTime = rtcData; /*Atualiza o horário*/
            xSemaphoreGive(RTCSemaphore);
        }
    }

    int ciclePosition(int position) {
        int64_t now = esp_timer_get_time();
        if (now - _lastCicleUpdate >= 25000 * abs(configs.cicleTime)) {
            _lastCicleUpdate = now;
            if (position >= _cicleLimit || _cicleIncrement >= _cicleLimit)
                _cicleIncrement = false;
            else if (position <= -_cicleLimit ||
                     _cicleIncrement <= -_cicleLimit)
                _cicleIncrement = true;
            if (_cicleIncrement) {
                _ciclePosition++;
            } else {
                _ciclePosition--;
            }
        }
        return _ciclePosition;
    }

    int sunPosition() {
        byte today[] = {dateTime.Second(), dateTime.Minute(),
                        dateTime.Hour(),   dateTime.Day(),
                        dateTime.Month(),  static_cast<byte>(dateTime.Year())};

        _dayCurrentTime = (double)dateTime.Hour() * 3600 +
                          (double)dateTime.Minute() * 60 +
                          (double)dateTime.Second();  // Hora atual em segundos

        if (lord.SunRise(today))
            _daySunRise =
                (double)today[tl_hour] * 3600 +
                (double)today[tl_minute] * 60;  // converte para segundos
        if (lord.SunSet(today))
            _daySunSet =
                (double)today[tl_hour] * 3600 +
                (double)today[tl_minute] * 60;  // converte para segundos

        return (map(_dayCurrentTime, _daySunRise, _daySunSet, 90.0, -90.0));
    }
};

TimeController timeInfo(0x68);
