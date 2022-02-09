#pragma once

#include <ESPmDNS.h>
#include <WiFi.h>

void wifiConnect(const char* ssid, const char* password, const char* dnsName = "lif") {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    MDNS.begin(dnsName);
}