#pragma once

#include <ESP32_FTPClient.h>

#include <credentials/credentials.hpp>

char* ftpServer   = FTP_SERVER_CREDENTIALS;
char* ftpUsername = FTP_USERNAME_CREDENTIALS;
char* ftpPassword = FTP_PASSWORD_CREDENTIALS;

ESP32_FTPClient ftp(ftpServer, ftpUsername, ftpPassword, 5000, 0);