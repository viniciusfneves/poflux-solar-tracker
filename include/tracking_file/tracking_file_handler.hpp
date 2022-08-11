#pragma once

#include <TimeController/TimeController.hpp>

SemaphoreHandle_t xTrackingFileSemaphore = xSemaphoreCreateMutex();

void writeDataToTrackingFile(uint32_t timestamp, int sunPosition, double lensAngle) {
    timestamp -= timeInfo.timezone() * 3600;  // Converte para o horário GMT
    xSemaphoreTake(xTrackingFileSemaphore, portMAX_DELAY);
    File trackFile = LITTLEFS.open("/tracking/tracking.csv", "a+");
    trackFile.print("REM1,");
    trackFile.print(timestamp);
    trackFile.print(",");
    trackFile.println(sunPosition);
    trackFile.print("REM2,");
    trackFile.print(timestamp);
    trackFile.print(",");
    trackFile.println(lensAngle);
    trackFile.print("REM3,");
    trackFile.print(timestamp);
    trackFile.println(",0");
    trackFile.print("REM4,");
    trackFile.print(timestamp);
    trackFile.println(",0");
    trackFile.close();
    xSemaphoreGive(xTrackingFileSemaphore);
}

void clearTrackingData() {
    xSemaphoreTake(xTrackingFileSemaphore, portMAX_DELAY);
    File trackFile = LITTLEFS.open("/tracking/tracking.csv", "w");
    trackFile.println("Channel name,Timestamp,Value");
    trackFile.close();
    xSemaphoreGive(xTrackingFileSemaphore);
}

int64_t ESPtimestamp     = 0;
int64_t lastESPtimestamp = 0;

void runDataLogger() {
    ESPtimestamp = esp_timer_get_time() / 1000;
    if (ESPtimestamp - lastESPtimestamp >= 10000) {
        writeDataToTrackingFile(dateTime.Epoch32Time(), timeInfo.sunPosition(), mpu.data.kalAngleX);
        lastESPtimestamp = ESPtimestamp;
    }
}