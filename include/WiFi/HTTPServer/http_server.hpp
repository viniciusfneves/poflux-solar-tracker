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

    server.on(
        "/config", HTTP_PATCH,
        [](AsyncWebServerRequest *request) {
            request->send(200, "text/plain", "CONFIG ENDPOINT...");
        },
        NULL,
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len,
           size_t index, size_t total) {
            DynamicJsonDocument  body(1024);
            DeserializationError error =
                deserializeJson(body, (const char *)data);
            if (error) request->send(400, "text/plain", "BAD REQUEST");
            serializeJsonPretty(body, Serial);
            request->send(200, "text/plain",
                          "JSON decoded and printed on the Serial port");
        });

    server.onNotFound([](AsyncWebServerRequest *request) {
        request->send(404, "text/plain", "NOT FOUND");
    });

    server.begin();
}