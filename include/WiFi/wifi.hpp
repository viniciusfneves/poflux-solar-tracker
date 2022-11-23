#pragma once

#include <ESPmDNS.h>
#include <WiFi.h>

#include <credentials/credentials.hpp>

/** @brief Connect to Wi-Fi network*/
void connectToWifiNetwork(const char* dnsName = "lif") {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.onEvent(
        [](system_event_id_t event, system_event_info_t info) {
            Serial.println("#-------IP-------#");
            Serial.println(WiFi.localIP());
            Serial.println("#----------------#");
        },
        system_event_id_t::SYSTEM_EVENT_STA_GOT_IP);

    WiFi.begin(WIFI_SSID_CREDENTIALS, WIFI_PASSWORD_CREDENTIALS);

    MDNS.begin(dnsName);
    vTaskDelay(3000);
}