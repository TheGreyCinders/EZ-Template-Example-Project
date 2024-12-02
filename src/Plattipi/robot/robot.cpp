#include "plattipi/robot/robot.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/llemu.hpp"
#include "Plattipi/utils.hpp"  // IWYU pragma: keep

ASSET(firstpath_txt);
ASSET(mogotoringstack_txt);
ASSET(ringstacktobackup_txt);
ASSET(rushTest_txt);

namespace plattipi {
namespace robot {
  Robot::Robot(subsystems::Intake& intake, subsystems::Conveyor& conveyor, subsystems::Arm& arm, subsystems::MogoMech& mogo_mech) : 
  m_intake{intake}, 
  m_conveyor{conveyor}, 
  m_arm{arm}, 
  m_mogo_mech{mogo_mech}
  {}
  
  //general methods
  void Robot::initialize() {

    pros::lcd::initialize();

    m_drive_train.initialize();
    m_intake.initialize();
    m_conveyor.initialize();
    m_arm.initialize();
    //mogo mech


  }

  void Robot::autonomous() {

    //set pose
    m_drive_train.llChassis.setPose(-50.809, 35.475, 270);
    // m_drive_train.llChassis.follow(firstpath_txt, 10, 5000, false, false);
    m_drive_train.llChassis.follow(rushTest_txt, 10, 5000, false, false);
    m_mogo_mech.toggleGrabbed();
    pros::delay(100);

    // //ring stacks
    // intakeIntake();
    // conveyorIntake();
    // periodic();
    // m_drive_train.llChassis.follow(mogotoringstack_txt, 12, 5000, true, false);
    // pros::delay(1500);

    // //drop mogo
    // m_drive_train.llChassis.turnToHeading(135, 2, {.maxSpeed = 120}, false);
    // m_mogo_mech.toggleGrabbed();
    // pros::delay(100);

    // //alliance ring stack
    // m_drive_train.llChassis.follow(ringstacktobackup_txt, 10, 5000, true, false);
    // pros::delay(50);
    // intakeStop();
    // conveyorStop();
    // periodic();

    // m_drive_train.llChassis.turnToHeading(90, 1000, {.maxSpeed = 120}, false);
    // m_drive_train.llChassis.moveToPoint(-61, 2.5, 1000, {.maxSpeed = 120}, false);
    // pros::delay(1000);
    // intakeIntake();
    // conveyorIntake();
    // periodic();
    // pros::delay(3000);
  }

  bool mogoGrabbed = false;

  void Robot::periodic() {

    //drive
    m_intake.periodic();
    m_conveyor.periodic();
    m_arm.periodic();
    //mogo mech


    // if (!mogoGrabbed && pointWithinRange(m_drive_train.llChassis.getPose().x, m_drive_train.llChassis.getPose().y, 0, 47, 12.5)) {
    //   m_mogo_mech.toggleGrabbed();
    //   mogoGrabbed = true;
    // }
    

    //prints
    pros::lcd::print(0, "X: %f", m_drive_train.getPoseX());
    pros::lcd::print(1, "Y: %f", m_drive_train.getPoseY());
    pros::lcd::print(2, "Theta: %f", m_drive_train.getPoseTheta());
    pros::lcd::print(4, "Speed: %f", m_drive_train.getDriveSpeed());
    pros::delay(10);
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