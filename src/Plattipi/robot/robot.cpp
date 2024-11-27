#include "plattipi/robot/robot.hpp"
namespace plattipi {
namespace robot {
  Robot::Robot(subsystems::DriveTrain& drive_train, subsystems::Intake& intake, subsystems::Conveyor& conveyor, subsystems::Arm& arm, subsystems::MogoMech& mogo_mech) : m_drive_train{drive_train}, m_intake{intake}, m_conveyor{conveyor}, m_arm{arm}, m_mogo_mech{mogo_mech} {}

  //general methods
  void Robot::initialize() {
    m_drive_train.initialize();
    m_intake.initialize();
    m_conveyor.initialize();
    m_arm.initialize();
    //mogo mech
  }

  void Robot::periodic() {
    //drive
    m_intake.periodic();
    m_conveyor.periodic();
    m_arm.periodic();
    //mogo mech
  }

//drive methods
  void Robot::driveSplitArcade(double forward_power, double turn_power){
    m_drive_train.driveSplitArcade(forward_power, turn_power);
  }

  void Robot::driveTank(double left_power, double right_power) {
    m_drive_train.driveTank(left_power, right_power);
  }

//intake methods
  void Robot::intakeIntake() {
    m_intake.intake();
  }

  void Robot::intakeOuttake() {
    m_intake.outtake();
  }

  void Robot::intakeStop() {
    m_intake.stop();
  }

//conveyor methods
  void Robot::conveyorIntake() {
    m_conveyor.intake();
  }

  void Robot::conveyorOuttake() {
    m_conveyor.outtake();
  }

  void Robot::conveyorStop() {
    m_conveyor.stop();
  }

//arm methods
  void Robot::armToggleOut() {
    m_arm.toggleOut();
  }
  
  void Robot::armToggleIn() {
    m_arm.toggleIn();
  }

//mogo methods
  void Robot::mogoToggle() {
    m_mogo_mech.toggleGrabbed();
  }
}  // namespace robot
}  // namespace plattipi