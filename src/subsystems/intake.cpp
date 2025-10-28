#include "main.h"
// brandon forge sucks at programming
void intakeInitialize() {
    intake.set_brake_mode(MOTOR_BRAKE_COAST);
}

void intakePeriodic() {
	if (controller.get_digital(DIGITAL_L1)){
        setIntakeSpeed(80);
    }
	else {
		setIntakeSpeed(0);
    }
}

void setIntakeSpeed(int speed){
    intake.move(speed);
}