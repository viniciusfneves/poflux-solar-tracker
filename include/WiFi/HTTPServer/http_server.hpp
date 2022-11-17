#pragma once

#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <WiFi.h>

#include <TimeController/TimeController.hpp>
#include <configurations/configurations.hpp>
#include <mpu/MPU.hpp>
#include <pid/PID_Controller.hpp>
#include <tracking_file/tracking_file_handler.hpp>

#define AUTH_USER "user"
#define AUTH_PASS "admin"

AsyncWebServer server(80);

void startHTTPServer() {
    LittleFS.begin();

    server.on("/", [](AsyncWebServerRequest *request) {
        request->redirect("/pof-lux");
    });

    server.serveStatic("/pof-lux", LittleFS, "/pages/pof-lux/index.html");

    server.on("/pof-lux/tracking", HTTP_GET,
              [](AsyncWebServerRequest *request) {
                  request->send(LittleFS, "/tracking/tracking.csv", "text/csv");
              });

    server.on("/pof-lux/clear_tracking", HTTP_GET,
              [](AsyncWebServerRequest *request) {
                  clearTrackingData();
                  request->redirect("/pof-lux");
              });

    server.serveStatic("/style.css", LittleFS, "/pages/pof-lux/style.css");

    server.serveStatic("/info_display.css", LittleFS,
                       "/pages/pof-lux/info_display.css");

    server.serveStatic("/scripts/monitor_script.js", LittleFS,
                       "/scripts/monitor_script.js");

    server.serveStatic("/scripts/websocket.js", LittleFS,
                       "/scripts/websocket.js");

    server.onNotFound([](AsyncWebServerRequest *request) {
        request->send(404, "text/plain", "NOT FOUND");
    });

    server.begin();
}