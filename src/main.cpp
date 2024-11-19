#include "main.h"

static constexpr int8_t LEFT_1_PORT = -1;
static constexpr int8_t LEFT_2_PORT = 2;
static constexpr int8_t LEFT_3_PORT = -3;
static constexpr int8_t LEFT_4_PORT = 4;

static constexpr int8_t RIGHT_4_PORT = -7;
static constexpr int8_t RIGHT_3_PORT = 8;
static constexpr int8_t RIGHT_2_PORT = -9;
static constexpr int8_t RIGHT_1_PORT = 10;

static constexpr int8_t INTAKE_PORT = 12;
static constexpr int8_t CONVEYOR_PORT = 5;
static constexpr int8_t CONVEYOR_2_PORT = 6;

static constexpr int8_t MOGO_PNEUMATICS = 'g';

static constexpr int8_t ARM_ONE = -14;
static constexpr int8_t ARM_TWO = 15;
static constexpr int8_t EXTENSION = -16;

static constexpr int8_t COLOR_SENSOR_PORT = 11;

static constexpr int16_t DRIVE_VELOCITY = 600;
static constexpr int16_t INTAKE_VELOCITY = -600;
static constexpr int16_t CONVEYOR_VELOCITY = 600;

static constexpr int8_t ALLIANCE = 1;
static constexpr int8_t OPP_ALLIANCE = 2;

static constexpr double ARM_DOWN = 0;
static constexpr double ARM_UP = 225;
static constexpr double ARM_ROTATE_SPEED = 85;

static constexpr double EXT_IN = 0;
static constexpr double EXT_STORE = 70;
static constexpr double EXT_OUT = 280;
static constexpr double EXT_SPEED = 600;
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

pros::Motor extension{EXTENSION, pros::v5::MotorCartridge::green, pros::v5::MotorUnits::degrees};
pros::Motor armleft{ARM_ONE, pros::v5::MotorCartridge::red, pros::v5::MotorUnits::degrees};
pros::Motor armright{ARM_TWO, pros::v5::MotorCartridge::red, pros::v5::MotorUnits::degrees};

std::vector<pros::Motor> leftDrive{{leftMotor1, leftMotor2, leftMotor3, leftMotor4}};
std::vector<pros::Motor> rightDrive{{rightMotor1, rightMotor2, rightMotor3, rightMotor4}};

std::vector<pros::Motor> armRotation{{armleft, armright}};

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
  for (auto motor : armRotation) {
    motor.set_brake_mode(MOTOR_BRAKE_HOLD);
  }
  extension.set_brake_mode(MOTOR_BRAKE_HOLD);
  conveyor.set_reversed(true, 0);
}

void disable() {}

void competition_initialize() {
  pros::lcd::initialize();
}

void autonomous() {}


enum ArmPosition{
  loading,
  storing,
  holding
};

void opcontrol() {
  bool arm_moving = false;
  bool rotation_moving = false;
  bool extension_move = false;
  double arm_start_time, arm_current_time;
  enum ArmPosition armPos=loading;
  enum ArmPosition pastArmPos=holding;
  int currentExtension;
  while (true) {
    currentExtension = extension.get_position();
    turn = (gp1.get_analog(ANALOG_LEFT_Y));
    power = (gp1.get_analog(ANALOG_RIGHT_X));

    leftPower = (power - turn) * -1;
    rightPower = (power + turn) * -1;

    drive.drive(leftPower, rightPower);

    if (gp1.get_digital(DIGITAL_R1)) {
      intake.move_velocity(INTAKE_VELOCITY);
      conveyor.move_velocity(CONVEYOR_VELOCITY);
      // autoChucker(CONVEYOR_VELOCITY);
    } else if (gp1.get_digital(DIGITAL_R2)) {
      intake.move_velocity(-INTAKE_VELOCITY);
      conveyor.move_velocity(-CONVEYOR_VELOCITY);
      // autoChucker(-CONVEYOR_VELOCITY);
    } else {
      intake.move_velocity(0);
      conveyor.move_velocity(0);
    }

    if (gp1.get_digital_new_press(DIGITAL_B)) {
      if (mogoGrabbed) {
        mogoPiston.set_value(false);
        mogoGrabbed = false;
      } else {
        mogoPiston.set_value(true);
        mogoGrabbed = true;
      }
    }

    // if (gp1.get_digital(DIGITAL_L1)&&currentExtension>-820) {
    //   extension.move_velocity(-200);
    // } else if (gp1.get_digital((DIGITAL_L2))&&currentExtension<0) {
    //   extension.move_velocity(200);
    // } else {
    //   extension.move_velocity(0);
    // }
    // if (gp1.get_digital(DIGITAL_RIGHT)) {
    //   extension.move_absolute(-820, -100);
    // } else if (gp1.get_digital_new_press(DIGITAL_DOWN)) {
    //   extension.move_absolute(0, 100);
    // }

//manual control
    // if (gp1.get_digital(DIGITAL_L1)) {
    //   for (auto motor : armRotation) {
    //     motor.move_absolute(ARM_UP, ARM_ROTATE_SPEED);
    //   }      
    //   extension.move_absolute(-820, -100);
    // } else if (gp1.get_digital_new_press(DIGITAL_L1)) {
    //   for (auto motor : armRotation) {
    //     motor.move_absolute(ARM_DOWN, -ARM_ROTATE_SPEED*0.75);
    //   }     
    //   extension.move_absolute(0, 100);
    // }

//macros
    if (gp1.get_digital_new_press(DIGITAL_L1)) {
      arm_moving=true;
      rotation_moving = true;
      extension_move = true;
      pastArmPos = armPos;
      if (armPos==loading){
        armPos=holding;
      }
      else if (armPos==holding || armPos==storing) {
        armPos = loading;
      }
    }

    if (gp1.get_digital_new_press(DIGITAL_L2)) {
      arm_moving = true;
      rotation_moving = true;
      extension_move = true;
      pastArmPos = armPos;
      if (armPos==loading) {
        armPos = storing;
      } else if (armPos==holding || armPos==storing) {
        armPos=loading;
      }
    }
    
    if (arm_moving=true) {

      //holding macros
      if (armPos == holding) {
        //loading-holding macro
        if (pastArmPos == loading) {
          if (rotation_moving == true) {
            for (auto motor : armRotation) {
              motor.move_absolute(ARM_UP, -ARM_ROTATE_SPEED);
            }
            rotation_moving = false;
          }
          if (extension_move == true && armRotation[0].get_position() > 125) {
            extension.move_absolute(EXT_OUT, -EXT_SPEED);
            extension_move = false;
          }
        }
        //storing-holding macro
        if (pastArmPos == holding) {
          //case is impossible
        }
      } 

      //loading macros
      else if (armPos == loading) {
        //holding-loading macro
        if (pastArmPos == holding) {
          if (extension_move == true) {
            extension.move_absolute(EXT_IN, EXT_SPEED);
            extension_move = false;
          }
          if (rotation_moving == true && extension.get_position() < 200) {
            for (auto motor : armRotation) {
              motor.move_absolute(ARM_DOWN, ARM_ROTATE_SPEED);
            }
            rotation_moving = false;
          }
        }
        //storing-loading macro
        if (pastArmPos == storing) {
          if (extension_move == true) {
            extension.move_absolute(EXT_IN, EXT_SPEED);
            extension_move = false;
          }
          rotation_moving = true;
        }
      }

      else if (armPos == storing) {
        //storing-holding macro
        if (pastArmPos == holding) {
          if (extension_move == true) {
            extension.move_absolute(EXT_STORE, EXT_SPEED);
            extension_move = false;
          }
          if (rotation_moving == true && extension.get_position() < 200) {
            for (auto motor : armRotation) {
              motor.move_absolute(ARM_DOWN, ARM_ROTATE_SPEED);
            }
            rotation_moving = false;
          }
        }
        //loading-storing macro
        if (pastArmPos == loading) {
          if (extension_move == true) {
            extension.move_absolute(EXT_STORE, EXT_SPEED);
            extension_move = false;
          }
          rotation_moving = true;
        }
      }

      //exit sequence
      if (!extension_move && !rotation_moving) {
        arm_moving = false;
      }
    }
    


    pros::lcd::set_text(0, std::to_string(extension.get_position()));
    pros::lcd::set_text(1, std::to_string(armleft.get_position()));
    pros::lcd::set_text(2, std::to_string(armright.get_position()));
    pros::lcd::set_text(3, std::to_string(armPos));
    pros::delay(10);
  
  }
}