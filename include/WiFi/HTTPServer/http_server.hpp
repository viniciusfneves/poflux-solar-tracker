#pragma once

#include <ESPAsyncWebServer.h>
#include <LittleFS.h>

#include <datalogger/datalogger.hpp>

AsyncWebServer server(80);

void startHTTPServer() {
    server.on("/tracking", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(LittleFS, DATALOGGER_FILE_PATH, "text/csv");
    });

    server.on("/clear_tracking", HTTP_DELETE,
              [](AsyncWebServerRequest *request) {
                  clearTrackingData();
                  request->send(200, "text/plain", "DONE");
              });

    server.on("/config", HTTP_PATCH, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", "CONFIG ENDPOINT...");
    });

    server.onNotFound([](AsyncWebServerRequest *request) {
        request->send(404, "text/plain", "NOT FOUND");
    });

    server.begin();
}