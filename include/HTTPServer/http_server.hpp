#pragma once

#include <ESPAsyncWebServer.h>

AsyncWebServer server(80);

void initHTTPServer() {
    server.on("/", [](AsyncWebServerRequest *request) { request->send(200, "text/plain", "POF LUX WEBSERVER ONLINE!"); });

    server.onNotFound([](AsyncWebServerRequest *request) { request->send(404, "text/plain", "NOT FOUND"); });

    server.begin();
}