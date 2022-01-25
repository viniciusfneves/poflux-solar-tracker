#pragma once

#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <WiFi.h>

#include <configurations/configurations.hpp>
#include <pid/PID_Controller.hpp>

#define AUTH_USER "user"
#define AUTH_PASS "admin"

AsyncWebServer server(80);

void initHTTPServer() {
    SPIFFS.begin();

    server.on("/", [](AsyncWebServerRequest *request) { request->redirect("/pof-lux"); });

    server.on("/settings", [](AsyncWebServerRequest *request) {
        if (request->hasParam("mode")) {
            const char *mode = request->getParam("mode")->value().c_str();
            if (strcmp(mode, "auto") == 0)
                configs.mode = Mode::Auto;
            if (strcmp(mode, "manual") == 0)
                configs.mode = Mode::Manual;
        }
        if (request->hasParam("setpoint")) {
            const int setpoint = request->getParam("setpoint")->value().toInt();
            configs.manualSetpoint = setpoint;
        }
        if (request->hasParam("kp")) {
            const double kp = request->getParam("kp")->value().toDouble();
            pid.setKp(kp);
        }
        if (request->hasParam("ki")) {
            const double ki = request->getParam("ki")->value().toDouble();
            pid.setKi(ki);
        }
        if (request->hasParam("kd")) {
            const double kd = request->getParam("kd")->value().toDouble();
            pid.setKd(kd);
        }

        request->send(200, "text/plain", "POF LUX: OK!");
    });

    server.on("/pof-lux", [](AsyncWebServerRequest *request) {
        AsyncWebServerResponse *response = request->beginResponse(SPIFFS, "/index.html");
        response->addHeader("local_ip", WiFi.localIP().toString());

        request->send(response);
    });

    server.onNotFound([](AsyncWebServerRequest *request) { request->send(404, "text/plain", "NOT FOUND"); });

    server.begin();
}