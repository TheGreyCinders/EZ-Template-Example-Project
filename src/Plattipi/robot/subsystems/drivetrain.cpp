#include "plattipi/robot/subsystems/drivetrain.hpp"
#include "drivetrain.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/motors.h"

namespace plattipi {
namespace robot {
namespace subsystems {
    DriveTrain::DriveTrain() : m_left_motor_group{m_left_motors, pros::v5::MotorGear::blue, pros::v5::MotorUnits::deg}, m_right_motor_group{m_right_motors, pros::v5::MotorGear::blue, pros::v5::MotorUnits::deg} {}

    void DriveTrain::initialize() {
        m_left_motor_group.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
        m_right_motor_group.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
        llChassis.calibrate();
    }
    void DriveTrain::drive(double left_power, double right_power) {
        m_leftPower = left_power;
        m_rightPower = right_power;
        llChassis.tank(left_power, right_power);
    }
    void DriveTrain::driveTank(double left_power, double right_power) {
        drive(left_power, right_power);
    }
    void DriveTrain::driveSplitArcade(double forward_power, double turn_power) {
        drive(((forward_power - turn_power) * -1), ((forward_power + turn_power) * -1));
    }

    double DriveTrain::getPoseX() {
        return llChassis.getPose().x;
    }

    double DriveTrain::getPoseY() {
        return llChassis.getPose().y;
    }

    double DriveTrain::getPoseTheta() {
        return llChassis.getPose().theta;
    }

    double DriveTrain::getDriveSpeed() {
        return m_leftPower;
    }

    


}  // namespace subsystems
}  // namespace robot
}  // namespace plattipi