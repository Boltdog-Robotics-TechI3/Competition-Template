#include "subsystems/intake.hpp"

void intakeInitialize()
{

    intakeMotors.set_brake_mode(pros::MotorBrake::coast);
}

void intakePeriodic()
{

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
    {
        intakeMotors.move(50);
    }
    else
    {
        intakeMotors.move(0);
    }
}