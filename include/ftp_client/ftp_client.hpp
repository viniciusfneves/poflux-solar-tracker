#pragma once

#include <ESP32_FTPClient.h>
#include <LITTLEFS.h>

#include <TimeController/TimeController.hpp>
#include <credentials/credentials.hpp>
#include <tracking_file/tracking_file_handler.hpp>

#define FTP_BUFFER_SIZE 1024
#define FTP_CONNECTION_TIMEOUT 5000  // ms

char* ftpServer   = FTP_SERVER_CREDENTIALS;
char* ftpUsername = FTP_USERNAME_CREDENTIALS;
char* ftpPassword = FTP_PASSWORD_CREDENTIALS;

ESP32_FTPClient ftp(ftpServer, ftpUsername, ftpPassword, FTP_CONNECTION_TIMEOUT, 0);

void upload_tracking_file_to_chronos() {
    String filename = "POFLUX_";
    filename.concat(dateTime.Year() - 2000);
    int month = dateTime.Month();
    if (month < 10) filename.concat(0);
    filename.concat(month);
    filename.concat(dateTime.Day());
    filename.concat(".csv");

    ftp.OpenConnection();
    ftp.ChangeWorkDir("novo");
    ftp.InitFile("Type I");
    ftp.NewFile(filename.c_str());
    xSemaphoreTake(xTrackingFileSemaphore, portMAX_DELAY);
    File trackFile = LITTLEFS.open("/tracking/tracking.csv", "a+");
    while (trackFile.available()) {
        unsigned char buf[FTP_BUFFER_SIZE];
        trackFile.read(buf, sizeof(buf));
        ftp.WriteData(buf, sizeof(buf));
    }
    ftp.CloseFile();

    trackFile.close();
    xSemaphoreGive(xTrackingFileSemaphore);
}