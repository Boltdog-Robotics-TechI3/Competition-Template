#pragma once
#include <cstdio>
#include <memory>
#include <string>
#include <vector>
#include "util/Logger.hpp"

using namespace std;

class CSV {
    private:
        shared_ptr<Logger> _logger;
        int width;
    /** Create a CSV File Logger
    * @param filename Name of the csv file
    * @param headers Name of each column. Length determines the amount of rows in the CSV.
    */
    public:
    CSV(std::string filename, std::vector<string> headers) : _logger(std::make_shared<Logger>(filename, true, false)) {
        width = headers.size();
        printData(headers);
    }

    void printData(std::vector<string> data) {
        string value = "";
        for (int i = 0; i < data.size(); i++) {
            value += data[i];
            value += ",";
        }
        _logger->Printfln(value.c_str());
    }
};