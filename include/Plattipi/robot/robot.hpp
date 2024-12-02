#ifndef _robot_hpp_
#define _robot_hpp_

#include "plattipi/robot/subsystems/DriveTrain.hpp"
#include "plattipi/robot/subsystems/Intake.hpp"
#include "plattipi/robot/subsystems/Arm.hpp"
#include "plattipi/robot/subsystems/MogoMech.hpp"
#include "Plattipi/robot/subsystems/Conveyor.hpp"

namespace plattipi {
namespace robot {
class Robot {
 private:
  subsystems::DriveTrain m_drive_train{};
  subsystems::Intake m_intake{};
  subsystems::Conveyor m_conveyor{};
  subsystems::Arm m_arm{};
  subsystems::MogoMech m_mogo_mech{};

 public:
  Robot(
    subsystems::Intake& intake,
    subsystems::Conveyor& conveyor,
    subsystems::Arm& arm,
    subsystems::MogoMech& mogo_mech
  );
  // Robot(
  //   subsystems::DriveTrain& drive_train, 
  //   subsystems::Intake& intake,
  //   subsystems::Conveyor& conveyor,
  //   subsystems::Arm& arm,
  //   subsystems::MogoMech& mogo_mech,
  //   configs::OrangeConfiguration& config
  // );
  // configs::BlueConfiguration blueConfig{};

  //general methods
  void initialize();
  void autonomous();
  void periodic();


  //drive methods
  void driveSplitArcade(double forward_power, double turn_power);
  void driveTank(double left_power, double right_power);

  //intake methods
  void intakeIntake();
  void intakeOuttake();
  void intakeStop();

  //conveyor methods
  void conveyorIntake();
  void conveyorOuttake();
  void conveyorStop();

  //arm methods
  void armToggleOut();
  void armToggleIn();

  //mogo methods
  void mogoToggle();

};
}  // namespace robot
}  // namespace plattipi
#endif