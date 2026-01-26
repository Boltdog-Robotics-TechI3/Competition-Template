#pragma once
#include <string>
#include <map>
#include <functional>

class AutoSelector {
    private:
        std::string currentAutoName;
        std::map<std::string, std::function<void()>> autos;

    public:
        std::string getCurrentAutoName() {
            return currentAutoName;
        }
        
        void setCurrentAutoName(std::string autoName){
            currentAutoName = autoName;
        }

        void addAuto(std::string key, std::function<void()> value) {
            autos[key] = value;
        }

        std::function<void()> getAuto(std::string key) {
            return autos[key];
        }

        std::function<void()> getCurrentAuto() {
            return autos[currentAutoName];
        }
};