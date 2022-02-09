#pragma once

#include <ESPAsyncWebServer.h>
#include <LITTLEFS.h>
#include <WiFi.h>

#include <TimeController/TimeController.hpp>
#include <configurations/configurations.hpp>
#include <mpu/MPU.hpp>
#include <pid/PID_Controller.hpp>

#define AUTH_USER "user"
#define AUTH_PASS "admin"

AsyncWebServer server(80);

void startHTTPServer() {
    LITTLEFS.begin();

    server.on("/", [](AsyncWebServerRequest *request) { request->redirect("/pof-lux"); });

    server.serveStatic("/pof-lux", LITTLEFS, "/pages/pof-lux/index.html").setAuthentication(AUTH_USER, AUTH_PASS);
    server.serveStatic("/style.css", LITTLEFS, "/pages/pof-lux/style.css");
    server.serveStatic("/info_display.css", LITTLEFS, "/pages/pof-lux/info_display.css");
    server.serveStatic("/monitor_script.js", LITTLEFS, "/pages/pof-lux/monitor_script.js");
    server.serveStatic("/scripts/websocket.js", LITTLEFS, "/scripts/websocket.js");

    server.onNotFound([](AsyncWebServerRequest *request) { request->send(404, "text/plain", "NOT FOUND"); });

    server.begin();
}