#include "plattipi/robot/subsystems/drivetrain.hpp"

namespace plattipi {
namespace robot {
namespace subsystems {
    DriveTrain::DriveTrain(std::vector<pros::Motor>& left_motors, std::vector<pros::Motor>& right_motors) : m_left_motors{left_motors}, m_right_motors{right_motors} {}
    void DriveTrain::initialize() {
        for(auto motor: m_left_motors){
            motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        }
        for(auto motor: m_right_motors){
            motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        }
    }
    void DriveTrain::drive(double left_power, double right_power) {
        for(auto motor: m_left_motors){
            motor.move(left_power);
        }
        for(auto motor:m_right_motors){
            motor.move(right_power);
        }   
    }
    void DriveTrain::driveTank(double left_power, double right_power) {
        drive(left_power, right_power);
    }
    void DriveTrain::driveSplitArcade(double forward_power, double turn_power) {
        drive(((forward_power - turn_power) * -1), ((forward_power + turn_power) * -1));
    }

}  // namespace subsystems
}  // namespace robot
}  // namespace plattipi