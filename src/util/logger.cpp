#include "util/Logger.hpp"
#include <cerrno>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <sys/stat.h>

#define MAX_LOG_AMOUNT 3

using namespace std;

/** Because renaming doesn't work, this file is used instead.
* @param og File to copy from
* @param destination File to copy file to. Does not have to already exist. If it does it will be overwritten.
*/
bool copyFile(const char* og, const char* destination) {
    bool sucess = false;
    FILE *ogFile = fopen(og, "r");
    FILE *destFile = fopen(destination, "w");

    if (ogFile == NULL || destFile == NULL) {
        //Ideally, we would have error logging here.
        goto close;
    }
    int c;
    while ((c = fgetc(ogFile)) != EOF)
    {
        fputc(c, destFile);
    }
    sucess = true;

    close:
        fclose(ogFile);
        fclose(destFile);
        return sucess;
}

Logger::Logger(string fileName, bool logToSD, bool logToSerial) {
    this->logToSDCard = logToSD;
    this->logToSerialOut = logToSerial;
    if (logToSD) {
        //Move all previous logs up 1 number
        for (int i = MAX_LOG_AMOUNT; i > 0; i--) {
            string filepath = "/usd/" + fileName + to_string(i - 1) + ".log";
            string nextpath = "/usd/" + fileName + to_string(i) + ".log";
            //Does the file exist?
            if ((this->logFile = fopen(filepath.c_str(), "r"))) {
                fclose(this->logFile);
                bool suc = copyFile(filepath.c_str(), nextpath.c_str());
            }
            else {
                break;
            }
        }
        string finalpath = "/usd/" + fileName + "0.log";
        this->logFile = fopen(finalpath.c_str(), "w");
    }
}

void Logger::Printf(const char* format, ...) {
    if (logToSerialOut) {
        va_list serial_args;
        va_start(serial_args, format);
        vprintf(format, serial_args);
        va_end(serial_args);
    }

    if (logToSDCard) {
        va_list args;
        va_start(args, format);
        vfprintf(logFile, format, args);
        va_end(args);
    }
}

void Logger::Printfln(const char* format, ...) {
    if (logToSerialOut) {
        va_list args;
        va_start(args, format);
        vprintf(format, args);
        va_end(args);
        printf("\r\n");
    }

    
    if (logToSDCard) {
        va_list args;
        va_start(args, format);
        vfprintf(logFile, format, args);
        va_end(args);
        fprintf(logFile, "\r\n");
    }
}