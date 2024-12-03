#include "plattipi/robot/robot.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/llemu.hpp"
#include "Plattipi/utils.hpp"  // IWYU pragma: keep
#include "pros/rtos.hpp"

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

  void Robot::intakeStart() {
    while (m_drive_train.llChassis.isInMotion()) {
      if (m_drive_train.llChassis.isInMotion()) {
        intakeIntake();
      }
      pros::delay(200);
    }
  }
  
  //general methods
  void Robot::initialize() {

    pros::lcd::initialize();

    m_drive_train.initialize();
    m_intake.initialize();
    m_conveyor.initialize();
    m_arm.initialize();
    //mogo mech

     pros::Task test {[=] (void) {
        intakeStart();
     }};
  }





  void Robot::autonomous() {

    //set pose
    m_drive_train.llChassis.setPose(-50.809, 35.475, 270);
    // m_drive_train.llChassis.follow(firstpath_txt, 10, 5000, false, false);
    m_drive_train.llChassis.follow(rushTest_txt, 10, 5000, false, false);
    m_mogo_mech.toggleGrabbed();
    pros::delay(100);

    //ring stacks
    intakeIntake();
    conveyorIntake();
    periodic();
    m_drive_train.llChassis.follow(mogotoringstack_txt, 12, 5000, true, false);
    pros::delay(1500);

    //drop mogo
    m_drive_train.llChassis.turnToHeading(135, 2, {.maxSpeed = 120}, false);
    m_mogo_mech.toggleGrabbed();
    pros::delay(100);

    //alliance ring stack
    m_drive_train.llChassis.follow(ringstacktobackup_txt, 10, 5000, true, false);
    pros::delay(50);
    intakeStop();
    conveyorStop();
    periodic();

    m_drive_train.llChassis.turnToHeading(90, 1000, {.maxSpeed = 120}, false);
    m_drive_train.llChassis.moveToPoint(-61, 2.5, 1000, {.maxSpeed = 120}, false);
    pros::delay(1000);
    intakeIntake();
    conveyorIntake();
    periodic();
    pros::delay(3000);
  }

  bool mogoGrabbed = false;

  void Robot::periodic() {
    //drive
    m_intake.periodic();
    m_conveyor.periodic();
    m_arm.periodic();
    //mogo mech

    if (sortingControl) {
      colorEjection();
    }

    pros::delay(10);
  }

  void Robot::colorEjection() {
    detection = m_intake.getHue();
    detectionColor = m_intake.getColor();

    if (!detected) {
      if (detectionColor == 1) {
        detected = true;
        detectTime = pros::millis();
      } else {
        m_intake.intakeIntake();
        m_conveyor.intake();
      }
    } else {
      pros::lcd::set_text(4, "yes");
      if ((detectTime + END_TIME) < pros::millis()) {
        detected = false;
      } else if ((detectTime + CONTINUE_TIME) < pros::millis()) {
        m_intake.intakeIntake();
        m_conveyor.intake();
      } else if ((detectTime + REVERSE_TIME) < pros::millis()) {
        m_intake.intakeOuttake();
        m_conveyor.outtake();
      }
    }
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
    m_intake.intakeIntake();
    // m_intake.conveyorIntake();
  }

  void Robot::intakeIntakeSort() {
    sortingControl = true;
  }

  void Robot::intakeOuttake() {
    m_intake.intakeOuttake();
    sortingControl = false;
    // m_intake.conveyorOuttake();
  }

  void Robot::intakeStop() {
    m_intake.intakeStop();
    sortingControl = false;
    // m_intake.conveyorStop();
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