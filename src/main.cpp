#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

static constexpr signed char LEFT_1_PORT = -1;
static constexpr signed char LEFT_2_PORT = 2;
static constexpr signed char LEFT_3_PORT = -3;
static constexpr signed char LEFT_4_PORT = 4;

static constexpr signed char RIGHT_4_PORT = -7;
static constexpr signed char RIGHT_3_PORT = 8;
static constexpr signed char RIGHT_2_PORT = -9;
static constexpr signed char RIGHT_1_PORT = 10;

static constexpr signed char INTAKE_PORT = -12;
static constexpr signed char CONVEYOR_PORT = -5;
static constexpr signed char CONVEYOR_2_PORT = 6;

static constexpr int8_t MOGO_PNEUMATICS = 'g';

static constexpr signed char ARM_ONE = -14;
static constexpr signed char ARM_TWO = 15;
static constexpr signed char EXTENSION = -16;

static constexpr signed char COLOR_SENSOR_PORT = 11;

static constexpr int8_t ALLIANCE = 1;
static constexpr int8_t OPP_ALLIANCE = 2;
// red 1
// blue 2

// pros::Motor leftMotor1{LEFT_1_PORT, pros::v5::MotorCartridge::blue, pros::v5::MotorUnits::degrees};
// pros::Motor leftMotor2{LEFT_2_PORT, pros::v5::MotorCartridge::blue, pros::v5::MotorUnits::degrees};
// pros::Motor leftMotor3{LEFT_3_PORT, pros::v5::MotorCartridge::blue, pros::v5::MotorUnits::degrees};
// pros::Motor leftMotor4{LEFT_4_PORT, pros::v5::MotorCartridge::blue, pros::v5::MotorUnits::degrees};

// pros::Motor rightMotor1{RIGHT_1_PORT, pros::v5::MotorCartridge::blue, pros::v5::MotorUnits::degrees};
// pros::Motor rightMotor2{RIGHT_2_PORT, pros::v5::MotorCartridge::blue, pros::v5::MotorUnits::degrees};
// pros::Motor rightMotor3{RIGHT_3_PORT, pros::v5::MotorCartridge::blue, pros::v5::MotorUnits::degrees};
// pros::Motor rightMotor4{RIGHT_4_PORT, pros::v5::MotorCartridge::blue, pros::v5::MotorUnits::degrees};

// std::vector<pros::Motor> leftDrive{{leftMotor1, leftMotor2, leftMotor3, leftMotor4}};
// std::vector<pros::Motor> rightDrive{{rightMotor1, rightMotor2, rightMotor3, rightMotor4}};

// pros::MotorGroup leftDrive({LEFT_1_PORT, LEFT_2_PORT, LEFT_3_PORT, LEFT_4_PORT}, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::deg);
// pros::MotorGroup rightDrive({RIGHT_1_PORT, RIGHT_2_PORT, RIGHT_3_PORT, RIGHT_4_PORT}, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::deg);

// std::vector<signed char> leftDrivePorts({LEFT_1_PORT, LEFT_2_PORT, LEFT_3_PORT, LEFT_4_PORT});
// std::vector<signed char> rightDrivePorts({RIGHT_1_PORT, RIGHT_2_PORT, RIGHT_3_PORT, RIGHT_4_PORT});

pros::Motor extension{EXTENSION, pros::v5::MotorCartridge::green, pros::v5::MotorUnits::degrees};
pros::Motor armleft{ARM_ONE, pros::v5::MotorCartridge::red, pros::v5::MotorUnits::degrees};
pros::Motor armright{ARM_TWO, pros::v5::MotorCartridge::red, pros::v5::MotorUnits::degrees};

std::vector<pros::Motor> armRotation{{armleft, armright}};

pros::Motor conveyor1{CONVEYOR_PORT, pros::v5::MotorCartridge::blue, pros::v5::MotorUnits::degrees};
pros::Motor conveyor2{CONVEYOR_2_PORT, pros::v5::MotorCartridge::blue, pros::v5::MotorUnits::degrees};
std::vector<pros::Motor> conveyorMotors{{conveyor1, conveyor2}};

pros::Motor intakeMotor{INTAKE_PORT, pros::v5::MotorCartridge::blue, pros::v5::MotorUnits::degrees};

pros::adi::DigitalOut mogoPiston(MOGO_PNEUMATICS);

// pros::Optical colorSensor(COLOR_SENSOR_PORT);

pros::Controller gp1(CONTROLLER_MASTER);

// Definitions
plattipi::robot::subsystems::DriveTrain drive{};
plattipi::robot::subsystems::Intake intake{intakeMotor};
plattipi::robot::subsystems::Conveyor conveyor{conveyorMotors};
plattipi::robot::subsystems::Arm arm{armleft, armright, extension};
plattipi::robot::subsystems::MogoMech mogo{mogoPiston};

plattipi::robot::Robot robot{intake, conveyor, arm, mogo};

enum DriverProfile{
  JACKSON,
  ETHAN,
  ASHER
};

enum DriverProfile controllingPerson;

// custom methods
// int detectColor(double hue) {
//   if (hue < 10 || hue > 350) {
//     return 1;
//   } else if (hue > 190 && hue < 230) {
//     return 2;
//   } else {
//     return 0;
//   }
// }

// int ringColor;
// bool chuckRing = false;
// int timer = 0;

// void autoChucker(int velocity) {
//   ringColor = detectColor(colorSensor.get_hue());
//   if (chuckRing == true) {
//     timer++;
//     if (timer > 90 && timer < 110) {
//       // conveyor.move_velocity(0);
//     } else if (timer > 110) {
//       chuckRing = false;
//       timer = 0;
//     }
//   } else {
//     // conveyor.move_velocity(velocity);
//     if (ringColor == OPP_ALLIANCE) {
//       chuckRing = true;
//     }
//   }
// }

// robot methods
void initialize() {
  pros::lcd::initialize();
  robot.initialize();
  controllingPerson=JACKSON;
}

void disable() {}

void competition_initialize() {
  pros::lcd::initialize();
}

void autonomous() {
  robot.autonomous();
}

void opcontrol() {
  bool change_driver=false;
  while (true) {
    //driver configs
    if (gp1.get_digital(DIGITAL_LEFT)==1&&gp1.get_digital(DIGITAL_UP)==1&&gp1.get_digital(DIGITAL_X)==1&&gp1.get_digital(DIGITAL_A)==1){
      if (change_driver==false) {
        if (controllingPerson==JACKSON){
          controllingPerson=ETHAN;
        } else if (controllingPerson==ETHAN){
          controllingPerson=ASHER;
        } else if (controllingPerson==ASHER){
          controllingPerson=JACKSON;
        }
        change_driver = true; 
      }
    } else {
      change_driver = false;
    }

    //drive controls
    if (controllingPerson==ETHAN||controllingPerson==ASHER)  {
      robot.driveSplitArcade(gp1.get_analog(ANALOG_RIGHT_X), gp1.get_analog(ANALOG_LEFT_Y));
    }
    else if (controllingPerson==JACKSON){
      robot.driveTank(gp1.get_analog(ANALOG_LEFT_Y), gp1.get_analog(ANALOG_RIGHT_Y));
    }

    //intake controls
    if (gp1.get_digital_new_press(DIGITAL_R1)) {
      robot.intakeIntake();
      robot.conveyorIntake();
      pros::lcd::set_text(2, "test");
    } else if (gp1.get_digital_new_press(DIGITAL_R2)) {
      robot.intakeOuttake();
      robot.conveyorOuttake();
    } else if (!gp1.get_digital(DIGITAL_R1) && !gp1.get_digital(DIGITAL_R2)){
      robot.intakeStop();
      robot.conveyorStop();
    }

    //good controls
    if (gp1.get_digital_new_press(DIGITAL_B)) {
      robot.mogoToggle();
    }

    if (gp1.get_digital_new_press(DIGITAL_L1)) {
      robot.armToggleOut();
    }

    if (gp1.get_digital_new_press(DIGITAL_L2)) {
      robot.armToggleIn();
    }

    if (gp1.get_digital_new_press(DIGITAL_DOWN)) {
      autonomous();
    }

    robot.periodic();  
  }
}