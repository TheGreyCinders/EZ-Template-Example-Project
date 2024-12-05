#include "plattipi/robot/robot.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/llemu.hpp"
#include "Plattipi/utils.hpp"  // IWYU pragma: keep
#include "pros/rtos.hpp"

//test paths
ASSET(firstpath_txt);
ASSET(mogotoringstack_txt);
ASSET(ringstacktobackup_txt);
ASSET(rushTest_txt);

//blue left
ASSET(BLstarttoalliancestake_txt);
ASSET(BLmogotoringstacks_txt);
ASSET(BLringstacktobackup_txt);
ASSET(BLbackuptocorner_txt);
ASSET(BLcornertobackup2_txt);
ASSET(BLbackup2tocenter_txt);

//blue right
ASSET(BRstarttomogo_txt);
ASSET(BRmogotocenter_txt);
ASSET(BRcentertobackup_txt);
ASSET(BRbackuptocorner_txt);

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

  void Robot::colorSortAuto() {
    while (true) {
      if (sortingControl && colorSortingToggle) {
        colorEjection();
      }
      pros::delay(10);
    }
  }
  
  //general methods
  void Robot::initialize(int alliance) {
    pros::lcd::initialize();

    m_drive_train.initialize();
    m_intake.initialize();
    m_conveyor.initialize();
    m_arm.initialize();
    //mogo mech

    // m_arm.toggleIn();

    m_alliance = alliance;

    // pros::Task test {[=] (void) {
    //   intakeStart();
    // }};

    pros::Task colorSortTask {[=] (void) {
      colorSortAuto();
    }};

    pros::Task periodicTask {[=] (void) {
      periodic();
    }};
  }



  void Robot::autoTest() {

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

  void Robot::autoBlueLeft() {
    colorSortingToggle = true;
    double cornerInX{61};
    double cornerInY{-59};
    double cornerOutX{56};
    double cornerOutY{-54};
    //set pose
    m_drive_train.llChassis.setPose(59, -12.5, 0);

    //start pose
    m_intake.intakeIntake();
    m_arm.toggleIn();

    //move to alliance stake
    m_drive_train.llChassis.moveToPoint(59, 0, 1250, {.maxSpeed = 65}, true);
    m_drive_train.llChassis.turnToHeading(272.5, 1250, {.maxSpeed = 65}, true);
    m_intake.intakeStop();
    m_drive_train.llChassis.moveToPoint(63, 0, 750, {.maxSpeed = 65}, true);

    m_drive_train.llChassis.waitUntilDone();

    //score on stake
    intakeIntake();
    conveyorIntake();
    pros::delay(650);
    intakeStop();
    conveyorStop();

    m_drive_train.llChassis.waitUntilDone();

    //grab mogo
    m_drive_train.llChassis.moveToPoint(56, 0, 1000, {.maxSpeed = 65}, true);
    m_drive_train.llChassis.waitUntilDone();
    m_drive_train.llChassis.turnToHeading(90, 1000, {.maxSpeed = 50}, true);
    m_drive_train.llChassis.moveToPoint(47, 1, 1000, {.forwards = false, .maxSpeed = 65}, true);
    m_drive_train.llChassis.waitUntil(8);
    m_mogo_mech.toggleGrabbed();

    m_drive_train.llChassis.waitUntilDone();

    //grab ring stacks
    sortingControl = true;

    pros::delay(250);

    m_drive_train.llChassis.follow(BLmogotoringstacks_txt, 10, 5000, true, true);
    m_drive_train.llChassis.waitUntilDone();

    pros::delay(1000);

    // //go to corner
    m_drive_train.llChassis.follow(BLringstacktobackup_txt, 10, 4000, false, true);
    m_drive_train.llChassis.follow(BLbackuptocorner_txt, 10, 1500, true, true);
    m_drive_train.llChassis.waitUntilDone();
    pros::delay(500);
    // m_drive_train.llChassis.moveToPoint(cornerInX, cornerInY, 800, {.maxSpeed = 50}, true);
    m_drive_train.llChassis.moveToPoint(cornerOutX, cornerOutY, 800, {.forwards = false, .maxSpeed = 50}, true);
    m_drive_train.llChassis.moveToPoint(cornerInX, cornerInY, 800, {.maxSpeed = 50}, true);
    m_drive_train.llChassis.moveToPoint(cornerOutX, cornerOutY, 800, {.forwards = false, .maxSpeed = 50}, true);
    m_drive_train.llChassis.moveToPoint(cornerInX, cornerInY, 800, {.maxSpeed = 50}, true);
    m_drive_train.llChassis.moveToPoint(cornerOutX, cornerOutY, 800, {.forwards = false, .maxSpeed = 50}, true);
    m_drive_train.llChassis.moveToPoint(cornerInX, cornerInY,800, {.maxSpeed = 50}, true);
    m_drive_train.llChassis.moveToPoint(cornerOutX, cornerOutY, 800, {.forwards = false, .maxSpeed = 50}, true);
    m_drive_train.llChassis.moveToPoint(cornerInX, cornerInY,800, {.maxSpeed = 50}, true);
    m_drive_train.llChassis.moveToPoint(cornerOutX, cornerOutY,800, {.forwards = false, .maxSpeed = 50}, true);
    m_drive_train.llChassis.moveToPoint(cornerInX, cornerInY, 800, {.maxSpeed = 50}, true);
    //go to center
    m_drive_train.llChassis.follow(BLcornertobackup2_txt, 10, 2000, false, true);
    m_drive_train.llChassis.waitUntilDone();
    intakeStop();
    conveyorStop();
    m_drive_train.llChassis.follow(BLbackup2tocenter_txt, 10, 2000, true, true);
  }

  void Robot::autoBlueRight() {
    //set pose
    m_drive_train.llChassis.setPose(53.25, 13.5, 90);

    //start movement
    m_drive_train.llChassis.follow(BRstarttomogo_txt, 10, 3, false, false);
    m_mogo_mech.toggleGrabbed();
    pros::delay(75);

    //center ring stack
    intakeIntakeSort();
    conveyorIntake();
    periodic();
    m_drive_train.llChassis.follow(BRmogotocenter_txt, 10, true, false);

    //go to corner
    m_drive_train.llChassis.follow(BRcentertobackup_txt, 10, 4, false, false);
    m_drive_train.llChassis.follow(BRbackuptocorner_txt, 10, 4, true, false);
    pros::delay(10000);
  }

  void Robot::autoRedLeft() {
    //set pose

  }

  void Robot::autoRedRight() {
    //set pose

  }

  bool mogoGrabbed = false;



  void Robot::periodic() {
    while (true) {
      //drive
      m_intake.periodic();
      m_conveyor.periodic();
      m_arm.periodic();
      //mogo mech
      
      pros::lcd::print(1, "Blue: %f", m_drive_train.llChassis.getPose().x);
      pros::lcd::print(2, "Red: %f", m_drive_train.llChassis.getPose().y);
      pros::lcd::print(3, "Color: %i", m_drive_train.llChassis.getPose().theta);
      pros::delay(10);
    }
  }

  void Robot::colorEjection() {
    detectionColor = m_intake.getColor();

    if (!detected) {
      if (detectionColor == m_alliance) {
        detected = true;
        detectTime = pros::millis();
      } else {
        m_intake.intakeIntake();
        m_conveyor.intake();
      }
    } else {
      if ((detectTime + END_TIME) < pros::millis()) {
        m_conveyor.retract();
        detected = false;
      } else {
        m_conveyor.extend();
        m_intake.intakeIntake();
        m_conveyor.intake();
      }
    }
    // detectionColor = m_intake.getColor();

    // if (!detected) {
    //   if (detectionColor == m_alliance) {
    //     detected = true;
    //     detectTime = pros::millis();
    //   } else {
    //     m_intake.intakeIntake();
    //     m_conveyor.intake();
    //   }
    // } else {
    //   if ((detectTime + END_TIME) < pros::millis()) {
    //     detected = false;
    //   } else if ((detectTime + CONTINUE_TIME) < pros::millis()) {
    //     m_intake.intakeIntake();
    //     m_conveyor.intake();
    //   } else if ((detectTime + REVERSE_TIME) < pros::millis()) {
    //     m_intake.intakeOuttake();
    //     m_conveyor.outtake();
    //   }
    // }
  }

  void Robot::toggleColorSorting() {
    colorSortingToggle = !colorSortingToggle;
  }

//drive methods
  void Robot::driveSplitArcade(double forward_power, double turn_power){
    m_drive_train.driveSplitArcade(-forward_power, turn_power);
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
    m_intake.intakeIntake();
    conveyorIntake();
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

  void Robot::test() {
    m_conveyor.extend();
  }
}  // namespace robot
}  // namespace plattipi