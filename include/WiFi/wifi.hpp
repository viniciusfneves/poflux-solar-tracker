#pragma once

#include <ESPmDNS.h>
#include <WiFi.h>

#include <credentials/credentials.hpp>

void connectToWifiNetwork(const char* dnsName = "lif") {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID_CREDENTIALS, WIFI_PASSWORD_CREDENTIALS);

    MDNS.begin(dnsName);
}