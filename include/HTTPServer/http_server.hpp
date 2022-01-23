#pragma once

#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>

#define AUTH_USER "user"
#define AUTH_PASS "admin"

AsyncWebServer server(80);

void initHTTPServer() {
    SPIFFS.begin();

    server.on("/", [](AsyncWebServerRequest *request) { request->redirect("/pof-lux"); });

    server.serveStatic("/pof-lux", SPIFFS, "/index.html").setAuthentication(AUTH_USER, AUTH_PASS);

    server.onNotFound([](AsyncWebServerRequest *request) { request->send(404, "text/plain", "NOT FOUND"); });

    server.begin();
}