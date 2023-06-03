#pragma once

#include <ESPAsyncWebServer.h>
#include <LittleFS.h>

#include <tracking_file/tracking_file_handler.hpp>

AsyncWebServer server(80);

void startHTTPServer() {
    server.on("/pof-lux/tracking", HTTP_GET,
              [](AsyncWebServerRequest *request) {
                  request->send(LittleFS, "/tracking/tracking.csv", "text/csv");
              });

    server.on("/pof-lux/clear_tracking", HTTP_DELETE,
              [](AsyncWebServerRequest *request) {
                  clearTrackingData();
                  request->send(200, "text/plain", "DONE");
              });

    server.onNotFound([](AsyncWebServerRequest *request) {
        request->send(404, "text/plain", "NOT FOUND");
    });

    server.begin();
}