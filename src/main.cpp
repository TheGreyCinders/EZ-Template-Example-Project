#include "main.h"

static constexpr int8_t LEFT_1_PORT = 1;
static constexpr int8_t LEFT_2_PORT = -2;
static constexpr int8_t LEFT_3_PORT = 3;
static constexpr int8_t LEFT_4_PORT = -4;

static constexpr int8_t RIGHT_4_PORT = 7;
static constexpr int8_t RIGHT_3_PORT = -8;
static constexpr int8_t RIGHT_2_PORT = 9;
static constexpr int8_t RIGHT_1_PORT = -10;

static constexpr int8_t INTAKE_PORT = 12;
static constexpr int8_t CONVEYOR_PORT = 5;
static constexpr int8_t CONVEYOR_2_PORT = 6;

static constexpr int8_t MOGO_PNEUMATICS = 7;

static constexpr int8_t ARM_ONE = 14;
static constexpr int8_t ARM_TWO = 15;
static constexpr int8_t in_out = 16;

static constexpr int8_t COLOR_SENSOR_PORT = 11;

static constexpr int16_t DRIVE_VELOCITY = 600;
static constexpr int16_t INTAKE_VELOCITY = -600;
static constexpr int16_t CONVEYOR_VELOCITY = 600;

static constexpr int8_t ALLIANCE = 1;
static constexpr int8_t OPP_ALLIANCE = 2;
// red 1
// blue 2

pros::Motor leftMotor1{LEFT_1_PORT, pros::v5::MotorCartridge::blue, pros::v5::MotorUnits::degrees};
pros::Motor leftMotor2{LEFT_2_PORT, pros::v5::MotorCartridge::blue, pros::v5::MotorUnits::degrees};
pros::Motor leftMotor3{LEFT_3_PORT, pros::v5::MotorCartridge::blue, pros::v5::MotorUnits::degrees};
pros::Motor leftMotor4{LEFT_4_PORT, pros::v5::MotorCartridge::blue, pros::v5::MotorUnits::degrees};

pros::Motor rightMotor1{RIGHT_1_PORT, pros::v5::MotorCartridge::blue, pros::v5::MotorUnits::degrees};
pros::Motor rightMotor2{RIGHT_2_PORT, pros::v5::MotorCartridge::blue, pros::v5::MotorUnits::degrees};
pros::Motor rightMotor3{RIGHT_3_PORT, pros::v5::MotorCartridge::blue, pros::v5::MotorUnits::degrees};
pros::Motor rightMotor4{RIGHT_4_PORT, pros::v5::MotorCartridge::blue, pros::v5::MotorUnits::degrees};

std::vector<pros::Motor> leftDrive{{leftMotor1, leftMotor2, leftMotor3, leftMotor4}};
std::vector<pros::Motor> rightDrive{{rightMotor1, rightMotor2, rightMotor3, rightMotor4}};

// Definitions
plattipi::robot::subsystems::DriveTrain drive{leftDrive, rightDrive};

pros::MotorGroup conveyor{
    {CONVEYOR_PORT, CONVEYOR_2_PORT},
    pros::v5::MotorCartridge::green,
    pros::v5::MotorUnits::degrees};

pros::Motor intake{
    INTAKE_PORT,
    pros::v5::MotorCartridge::blue,
    pros::v5::MotorUnits::degrees};

pros::Motor womp{ARM_ONE};

pros::adi::DigitalOut mogoPiston(MOGO_PNEUMATICS);

pros::Optical colorSensor(COLOR_SENSOR_PORT);

pros::Controller gp1(CONTROLLER_MASTER);

bool mogoGrabbed = false;

// custom methods
double convertToVelocity(double joystick) {
  return (joystick * DRIVE_VELOCITY) / 127;
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

int turn, power, leftPower, rightPower;

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

// robot methods
void initialize() {
  pros::lcd::initialize();
  drive.initialize();
  conveyor.set_brake_mode_all(MOTOR_BRAKE_BRAKE);
  intake.set_brake_mode_all(MOTOR_BRAKE_COAST);

  conveyor.set_reversed(true, 0);
}

void disable() {}

void competition_initialize() {
  pros::lcd::initialize();
}

void autonomous() {}

void opcontrol() {
  while (true) {
    turn = convertToVelocity(gp1.get_analog(ANALOG_LEFT_Y)) / 2;
    power = convertToVelocity(gp1.get_analog(ANALOG_RIGHT_X)) / 2;

    leftPower = (power + turn) * -1;
    rightPower = (power - turn) * -1;

    drive.drive(leftPower, rightPower);

    // leftDrive.move_velocity((power + (turn)) * -1);
    // rightDrive.move_velocity((power - (turn)) *  -1);

    if (gp1.get_digital(DIGITAL_A)) {
      intake.move_velocity(INTAKE_VELOCITY);
      conveyor.move_velocity(CONVEYOR_VELOCITY);
      // autoChucker(CONVEYOR_VELOCITY);
    } else if (gp1.get_digital(DIGITAL_B)) {
      intake.move_velocity(-INTAKE_VELOCITY);
      conveyor.move_velocity(-CONVEYOR_VELOCITY);
      // autoChucker(-CONVEYOR_VELOCITY);
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