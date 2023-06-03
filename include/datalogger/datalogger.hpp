#pragma once

#include <LittleFS.h>

#include <TimeController/TimeController.hpp>

#define DATALOGGER_FILE_PATH "/tracking.csv"

void clearTrackingData() {
    File trackFile = LittleFS.open(DATALOGGER_FILE_PATH, "w", true);
    if (!trackFile) return;
    trackFile.println("Channel name,Timestamp,Value");
    trackFile.close();
}

void writeDataToTrackingFile(int64_t EPOCHtimestamp, int sunPosition,
                             double lensAngle) {
    File trackFile = LittleFS.open(DATALOGGER_FILE_PATH, "a+", false);
    if (!trackFile) return;
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
}

void dataloggerTask(void* _) {
    for (;;) {
        writeDataToTrackingFile(timeInfo.datetime(), timeInfo.sunPosition(),
                                mpu.data.kalAngleX);
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}