
#include "logger.hpp"
#include <memory>
using namespace std;

class PiLogger {
    private:
        shared_ptr<Logger> logger;
    public:
        PiLogger() : logger(std::make_shared<Logger>("idk", false, true)) {}

        void LogSensorFloat(string sensorName, float value) {
            logger->Printfln("{\"type\":\"sensor\", \"name\":\"%s\", \"value\":%f}", sensorName.c_str(), value);
        }

        void LogString(string value) {
            logger->Printfln("{\"type\":\"stdout\", \"value\":\"%s\"}", value.c_str());
        }
};