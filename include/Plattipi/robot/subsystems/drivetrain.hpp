#pragma once

#include "pros/motor_group.hpp"
namespace plattipi {
namespace robot {
namespace subsystems {
class DriveTrain {
 private:
  std::vector<pros::Motor> m_left_motors{};

  std::vector<pros::Motor> m_right_motors{};

  public:
  DriveTrain();
  DriveTrain(std::vector<pros::Motor>& left_motors, std::vector<pros::Motor>& right_motors);

  void drive (double left_power, double right_power);
  void initialize();


};
}
}
}
