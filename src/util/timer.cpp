#include "api.h"
#include <climits>
#include "util/timer.hpp"

Timer::Timer(int time, callback_function cb) : time(time), callback(cb) {
    pros::delay(100); // Small delay to ensure task is properly initialized
}

void Timer::start() {
    if (!running) {
        pros::lcd::print(5, "Timer Start Running %d", pros::millis());
        task.resume();
        running = true;
    }
}

void Timer::stop() {
    running = false;
}
