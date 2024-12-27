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

  bool sortingControl{false};
  bool sorting{false};
  bool detected{false};
  double detectTime{0};
  double detection{0};
  int detectionColor{0};
  bool colorSortingToggle{true};

  int m_alliance{0};

  static constexpr double REVERSE_TIME{115};
  static constexpr double CONTINUE_TIME{200};
  static constexpr double END_TIME{400};

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
  void initialize(int alliance);
  void colorSortAuto();
  void autoTest();
  void autoBlueLeft();
  void autoBlueRight();
  void autoRedLeft();
  void autoRedRight();
  void autoSkillsBlue();
  void periodic();

  void colorEjection();
  void colorEjectionTest();

  void toggleColorSorting();


  //drive methods
  void driveSplitArcade(double forward_power, double turn_power);
  void driveTank(double left_power, double right_power);

  //intake methods
  void intakeIntake();
  void intakeIntakeSort();
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
  void testToggle();

  //auto methods
  void intakeStart();

  void test();
  void test2();

};
}  // namespace robot
}  // namespace plattipi
#endif