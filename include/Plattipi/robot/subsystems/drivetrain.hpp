#pragma once

#include "lemlib/chassis/chassis.hpp"
#include "pros/motor_group.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
namespace plattipi {
namespace robot {
namespace subsystems {
class DriveTrain {
 private:
  //ports
  // static constexpr signed char LEFT_1_PORT{-1};
  // static constexpr signed char LEFT_2_PORT{2};
  // static constexpr signed char LEFT_3_PORT{-3};
  // static constexpr signed char LEFT_4_PORT{4};

  // static constexpr signed char RIGHT_4_PORT{7};
  // static constexpr signed char RIGHT_3_PORT{-8};
  // static constexpr signed char RIGHT_2_PORT{9};
  // static constexpr signed char RIGHT_1_PORT{-10};

  static constexpr signed char LEFT_1_PORT{7};
  static constexpr signed char LEFT_2_PORT{-8};
  static constexpr signed char LEFT_3_PORT{9};
  static constexpr signed char LEFT_4_PORT{-10};

  static constexpr signed char RIGHT_4_PORT{-1};
  static constexpr signed char RIGHT_3_PORT{2};
  static constexpr signed char RIGHT_2_PORT{-3};
  static constexpr signed char RIGHT_1_PORT{4};

  static constexpr signed char IMU_PORT{17};

  static constexpr signed char HORIZONTAL_ENCODER{18};
  static constexpr signed char VERTICAL_ENCODER{19};

  static constexpr double TRACK_WIDTH{11.625};
  static constexpr double OMNI_DIAMETER{2.5};
  static constexpr double HORIZONTAL_DRIFT {2};

  static constexpr double HORIZONTAL_OFFSET{-2};
  static constexpr double VERTICAL_OFFSET{3.5};
  static constexpr double TRACKING_WHEEL_DIAMETER{2.25};

  std::vector<signed char> m_left_motors{LEFT_1_PORT, LEFT_2_PORT, LEFT_3_PORT, LEFT_4_PORT};
  std::vector<signed char> m_right_motors{RIGHT_1_PORT, RIGHT_2_PORT, RIGHT_3_PORT, RIGHT_4_PORT};

  pros::MotorGroup m_left_motor_group{m_left_motors, pros::v5::MotorGear::blue, pros::v5::MotorUnits::deg};
  pros::MotorGroup m_right_motor_group{m_right_motors, pros::v5::MotorGear::blue, pros::v5::MotorUnits::deg};

  double m_leftPower;
  double m_rightPower;

  pros::IMU imu{IMU_PORT};

  pros::Rotation horizontalEncoder{HORIZONTAL_ENCODER};
  pros::Rotation verticalEncoder{VERTICAL_ENCODER};

  lemlib::Drivetrain m_drivetrain{&m_left_motor_group, &m_right_motor_group, TRACK_WIDTH, OMNI_DIAMETER, 600, HORIZONTAL_DRIFT};

  lemlib::TrackingWheel horizontalWheel{&horizontalEncoder, TRACKING_WHEEL_DIAMETER, HORIZONTAL_OFFSET};
  lemlib::TrackingWheel verticalWheel{&verticalEncoder, TRACKING_WHEEL_DIAMETER, VERTICAL_OFFSET};

  lemlib::OdomSensors sensors{
    &verticalWheel,
    nullptr,
    &horizontalWheel,
    nullptr,
    &imu
  };

  lemlib::ControllerSettings lateral_controller{
    10,
    0,
    5,
    0,
    0,
    0,
    0,
    0,
    0
  };

  lemlib::ControllerSettings angular_controller{
    3,
    0,
    5,
    0,
    0,
    0,
    0,
    0,
    0
  };



  public:
  DriveTrain();
  DriveTrain(std::vector<signed char>& left_motors, std::vector<signed char>& right_motors);

  lemlib::Chassis llChassis{
    m_drivetrain,
    lateral_controller,
    angular_controller,
    sensors
  };

  void initialize();

  //driving methods
  void drive(double left_power, double right_power);
  void driveTank(double left_power, double right_power);
  void driveSplitArcade(double forward_power, double turn_power);
  pros::MotorGroup getLeftMotors();
  pros::MotorGroup getRightMotors();
  double getDriveSpeed();

  //odom methods
  double getPoseX();
  double getPoseY();
  double getPoseTheta();

  lemlib::Chassis getChassis();
};
}
}
}
