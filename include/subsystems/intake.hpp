#pragma once

#include "main.h"

// Initializes the intake motor, setting brake mode
void intakeInitialize();

// Polls the controller and determines motor state
void intakePeriodic();

// Sets the speed of the intake motor to the specified parameter
void setIntakeSpeed(int speed);