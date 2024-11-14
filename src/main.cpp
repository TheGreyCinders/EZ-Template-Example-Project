#include "main.h"
#define LEFT_1_PORT -1
#define LEFT_2_PORT -2
#define LEFT_3_PORT -3
#define LEFT_4_PORT -4

#define RIGHT_4_PORT 6
#define RIGHT_3_PORT 8
#define RIGHT_2_PORT 9
#define RIGHT_1_PORT 10

#define INTAKE_PORT 12
#define CONVEYOR_PORT -5
#define CONVEYOR_2_PORT -7

#define MOGO_PNEUMATICS 'h'

#define COLOR_SENSOR_PORT 17

#define DRIVE_VELOCITY 600
#define INTAKE_VELOCITY -600
#define CONVEYOR_VELOCITY 600

#define ALLIANCE 1
#define OPP_ALLIANCE 2
//red 1
//blue 2



//Definitions
pros::MotorGroup leftDrive (
	{LEFT_1_PORT, LEFT_2_PORT, LEFT_3_PORT, LEFT_4_PORT}, 
	pros::v5::MotorCartridge::blue, 
	pros::v5::MotorUnits::degrees
);

pros::MotorGroup rightDrive (
	{RIGHT_1_PORT, RIGHT_2_PORT, RIGHT_3_PORT, RIGHT_4_PORT},
	pros::v5::MotorCartridge::blue,
	pros::v5::MotorUnits::degrees
);

pros::MotorGroup conveyor (
	{CONVEYOR_PORT, CONVEYOR_2_PORT}, 
	pros::v5::MotorCartridge::green, 
	pros::v5::MotorUnits::degrees
);

pros::Motor intake (
	INTAKE_PORT,
	pros::v5::MotorCartridge::blue,
	pros::v5::MotorUnits::degrees
);

pros::ADIDigitalOut mogoPiston (MOGO_PNEUMATICS);

pros::Optical colorSensor(COLOR_SENSOR_PORT);

pros::Controller gp1(CONTROLLER_MASTER);

bool mogoGrabbed = false;

//custom methods
double convertToVelocity(double joystick) {
	return (joystick*DRIVE_VELOCITY)/127;
}

int detectColor(double hue) {
	if (hue < 10 || hue > 350) {
		return 1;
	} else if (hue > 190 && hue < 230) {
		return 2;
	} else {
		return 0;
	}
}

int ringColor;
bool chuckRing = false;
int timer = 0;

void autoChucker(int velocity) {

	ringColor = detectColor(colorSensor.get_hue());
	if (chuckRing == true) {
		timer++;
		if (timer > 90 && timer < 110) {
			conveyor.move_velocity(0);
		} else if (timer > 110) {
			chuckRing = false;
			timer = 0;
		}
	} else {
		conveyor.move_velocity(velocity);
		if (ringColor == OPP_ALLIANCE) {
			chuckRing = true;
		}
	}

}

//robot methods
void initialize() {
	pros::lcd::initialize();
	leftDrive.set_brake_mode_all(MOTOR_BRAKE_BRAKE);
	rightDrive.set_brake_mode_all(MOTOR_BRAKE_BRAKE);
	conveyor.set_brake_mode_all(MOTOR_BRAKE_BRAKE);
	intake.set_brake_mode_all(MOTOR_BRAKE_COAST);

	leftDrive.set_reversed(false, 0);
	leftDrive.set_reversed(true, 1);
	leftDrive.set_reversed(false, 2);
	leftDrive.set_reversed(true, 3);

	rightDrive.set_reversed(true, 0);
	rightDrive.set_reversed(false, 1);
	rightDrive.set_reversed(true, 2);
	rightDrive.set_reversed(false, 3);
	
	conveyor.set_reversed(true, 0);
}

void disable() {}


void competition_initialize() {
	pros::lcd::initialize();
}

void autonomous() {}

void opcontrol() {
	while (true) {
		int turn = convertToVelocity(gp1.get_analog(ANALOG_LEFT_Y))/2;
		int power = convertToVelocity(gp1.get_analog(ANALOG_RIGHT_X))/2;

		leftDrive.move_velocity((power + (turn)) * -1);
		rightDrive.move_velocity((power - (turn)) *  -1);

		if (gp1.get_digital(DIGITAL_A)) {
			intake.move_velocity(INTAKE_VELOCITY);
			autoChucker(CONVEYOR_VELOCITY);
		} else if (gp1.get_digital(DIGITAL_B)) {
			intake.move_velocity(-INTAKE_VELOCITY);
			autoChucker(-CONVEYOR_VELOCITY);
		} else {
			intake.move_velocity(0);
			conveyor.move_velocity(0);
		}

		if (gp1.get_digital_new_press(DIGITAL_R1)) {
			if (mogoGrabbed) {
				mogoPiston.set_value(false);
				mogoGrabbed = false;
			} else {
				mogoPiston.set_value(true);
				mogoGrabbed = true;
			}
		}

		pros::lcd::set_text(1, std::to_string(detectColor(colorSensor.get_hue())));
		pros::lcd::set_text(2, std::to_string(timer));
		pros::lcd::set_text(3, std::to_string(chuckRing));
		pros::lcd::set_text(4, std::to_string(colorSensor.get_hue()));
		pros::delay(2);
	}
}