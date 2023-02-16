#pragma once

#include <LittleFS.h>

#include <TimeController/TimeController.hpp>

SemaphoreHandle_t xTrackingFileSemaphore = xSemaphoreCreateMutex();

void writeDataToTrackingFile(int64_t EPOCHtimestamp, int sunPosition,
                             double lensAngle) {
    xSemaphoreTake(xTrackingFileSemaphore, portMAX_DELAY);
    File trackFile = LittleFS.open("/tracking/tracking.csv", "a+");
    trackFile.print("REM1,");
    trackFile.print(EPOCHtimestamp);
    trackFile.print(",");
    trackFile.println(sunPosition);
    trackFile.print("REM2,");
    trackFile.print(EPOCHtimestamp);
    trackFile.print(",");
    trackFile.println(lensAngle);
    trackFile.print("REM3,");
    trackFile.print(EPOCHtimestamp);
    trackFile.println(",0");
    trackFile.print("REM4,");
    trackFile.print(EPOCHtimestamp);
    trackFile.println(",0");
    trackFile.close();
    xSemaphoreGive(xTrackingFileSemaphore);
}

void clearTrackingData() {
    xSemaphoreTake(xTrackingFileSemaphore, portMAX_DELAY);
    File trackFile = LittleFS.open("/tracking/tracking.csv", "w");
    trackFile.println("Channel name,Timestamp,Value");
    trackFile.close();
    xSemaphoreGive(xTrackingFileSemaphore);
}

int64_t ESPtimestamp     = 0;
int64_t lastESPtimestamp = 0;

void runDataLogger() {
    ESPtimestamp = esp_timer_get_time() / 1000;
    if (ESPtimestamp - lastESPtimestamp >= 10000) {
        writeDataToTrackingFile(timeInfo.datetime(), timeInfo.sunPosition(),
                                mpu.data.kalAngleX);
        lastESPtimestamp = ESPtimestamp;
    }
}