#ifndef _robot_hpp_
#define _robot_hpp_

#include "plattipi/robot/subsystems/DriveTrain.hpp"
namespace plattipi {
namespace robot {
class Robot {
 private:
  subsystems::DriveTrain m_drive_train{};

 public:
  Robot(subsystems::DriveTrain& drive_train);

  void drive(double left_power, double right_power);
};
}  // namespace robot
}  // namespace plattipi
#endif