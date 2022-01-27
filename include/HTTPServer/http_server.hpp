#pragma once

#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <WiFi.h>

#include <TimeController/TimeController.hpp>
#include <configurations/configurations.hpp>
#include <mpu/MPU.hpp>
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

    server.serveStatic("/pof-lux", SPIFFS, "/index.html").setAuthentication(AUTH_USER, AUTH_PASS);

    server.on("/dump", [](AsyncWebServerRequest *request) {
        if (!request->authenticate(AUTH_USER, AUTH_PASS))
            return request->requestAuthentication();
        String response = "ip: " + WiFi.localIP().toString() + "\n";
        response += "RTC: ";
        response += dateTime.Day();
        response += "/";
        response += dateTime.Month();
        response += "/";
        response += dateTime.Year();
        response += "  ";
        response += dateTime.Hour();
        response += ":";
        response += dateTime.Minute();
        response += ":";
        response += dateTime.Second();
        response += "\nkp: ";
        response += pid.getKp();
        response += " | ki: ";
        response += pid.getKi();
        response += " | kd: ";
        response += pid.getKd();
        response += "\nmode: ";
        if (configs.mode == Mode::Auto)
            response += "auto";
        else
            response += "manual";
        response += "\nMPU : ";
        response += mpuData.roll;
        response += "\nAuto Setpoint: ";
        response += timeInfo.sunPosition();
        response += "\nManual Setpoint: ";
        response += configs.manualSetpoint;

        request->send(200, "text/plain", response);
    });

    server.onNotFound([](AsyncWebServerRequest *request) { request->send(404, "text/plain", "NOT FOUND"); });

    server.begin();
}