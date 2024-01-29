#pragma once

#include <RtcDS3231.h>
#include <RtcDateTime.h>
#include <TimeLord.h>  //Biblioteca que indica posição do Sol. Pode ser removida e substituida por algo mais simples?
#include <Wire.h>      //Biblioteca para manipulação do protocolo I2C
#include <time.h>

// Criação do objeto do tipo DS3231
RtcDS3231<TwoWire> rtc(Wire);

TimeLord lord;

class TimeController {
   private:
    // Criação do objeto do tipo RTCDateTime iniciando com tempo do sistema
    RtcDateTime _datetime = RtcDateTime(__DATE__, __TIME__);

    int _rtcAddress;

    double _longitude = -43.2311486;  // Configurado para Rio de Janeiro
    double _latitude  = -22.8613427;

    double _dayCurrentTime;
    double _daySunRise;
    double _daySunSet;

    /**
     * @brief Sets the internal clock on the ESP32
     */
    void _setInternalRTC(const int64_t& unix_time) {
        timeval _date;
        _date.tv_sec  = unix_time;
        _date.tv_usec = 0;
        settimeofday(&_date, NULL);
    }

   public:
    TimeController(int RTCAddress = 0x68) { _rtcAddress = RTCAddress; }

    void init() {
        rtc.Begin();  // Inicialização do RTC DS3231
        callRTC();
        _setInternalRTC(RTCdatetime());
        lord.TimeZone(_timezone * 60);  // Envio de informações para TimeLord
        lord.Position(_latitude, _longitude);
    }

    /**
     * @brief Set the value on the RTC module with the UNIX time
     * @param unix_time UNIX time
     */
    void setDatetime(const int64_t& unix_time) {
        _setInternalRTC(unix_time);
        _datetime.InitWithUnix64Time(unix_time);
        rtc.SetDateTime(_datetime);
    }

    /**
     * @brief Update the runtime datetime value from the RTC module
     */
    void callRTC() {
        RtcDateTime rtcData = rtc.GetDateTime();
        if (rtcData.IsValid()) {
            _datetime = rtcData; /*Atualiza o horário*/
        }
    }

    /**
     * @brief Reads the ESP32 internal RTC
     * @returns the datetime value in UNIX format
     */
    int64_t datetime() { return time(NULL); }

    /**
     * @brief Reads the external RTC Module
     * @returns the datetime value in UNIX format
     */
    int64_t RTCdatetime() { return _datetime.Unix64Time(); }

    /**
     * @brief Calculates the theoretical position of greatest incidence of solar
     * rays
     * @returns the sun position in degrees
     */
    int sunPosition() {
        byte today[] = {_datetime.Second(), _datetime.Minute(),
                        _datetime.Hour(),   _datetime.Day(),
                        _datetime.Month(),  static_cast<byte>(_datetime.Year())};

        _dayCurrentTime = (double)_datetime.Hour() * 3600 +
                          (double)_datetime.Minute() * 60 +
                          (double)_datetime.Second();  // Hora atual em segundos

        if (lord.SunRise(today))
            _daySunRise = (double)today[tl_hour] * 3600 +
                          (double)today[tl_minute] * 60;  // converte para segundos
        if (lord.SunSet(today))
            _daySunSet = (double)today[tl_hour] * 3600 +
                         (double)today[tl_minute] * 60;  // converte para segundos

        return (map(_dayCurrentTime, _daySunRise, _daySunSet, 90.0, -90.0));
    }
};

TimeController timeInfo(0x68);
