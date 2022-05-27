#pragma once

SemaphoreHandle_t xTrackingFileSemaphore = xSemaphoreCreateMutex();

void writeDataToTrackingFile(uint32_t timestamp, int sunPosition, double lensAngle) {
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