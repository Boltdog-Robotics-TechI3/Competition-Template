#pragma once
#include <cstdio>
#include <string>

class Logger {
    private:
        bool logToSDCard;
        bool logToSerialOut;
        FILE *logFile;
    public:
        ~Logger() {
            if (logFile) fclose(logFile);
        }
        
        Logger(std::string filename, bool logToSD, bool logToSerial = false);

        /** Prints a formatted string to the log file.
        * @param format Formatting string, like the regular printf().
        * @remarks can accept any number of items to be formatted in printf
        */
        void Printf(const char* format, ...);

        
        /** Prints a formatted string to the log file, with an endline at the end.
        * @param format Formatting string, like the regular printf().
        * @remarks can accept any number of items to be formatted in printf
        */
        void Printfln(const char* format, ...);
};