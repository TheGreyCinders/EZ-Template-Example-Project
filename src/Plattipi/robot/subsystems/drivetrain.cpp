#include "plattipi/robot/subsystems/drivetrain.hpp"

namespace plattipi {
namespace robot {
namespace subsystems {
    DriveTrain::DriveTrain(){}
DriveTrain::DriveTrain(std::vector<pros::Motor>& left_motors, std::vector<pros::Motor>& right_motors) : m_left_motors{left_motors}, m_right_motors{right_motors} {}
void DriveTrain::drive(double left_power, double right_power){
    for(auto motor: m_left_motors){
        motor.move(left_power);
    }
    for(auto motor:m_right_motors){
        motor.move(right_power);
    }
}
}  // namespace subsystems
}  // namespace robot
}  // namespace plattipi