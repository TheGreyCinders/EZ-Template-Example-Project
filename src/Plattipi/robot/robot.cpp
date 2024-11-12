#include "plattipi/robot/robot.hpp"
namespace plattipi {
namespace robot {
Robot::Robot(subsystems::DriveTrain& drive_train) : m_drive_train{drive_train} {}

void Robot::drive(double left_power, double right_power){
  m_drive_train.drive(left_power, right_power);
}
}  // namespace robot
}  // namespace plattipi