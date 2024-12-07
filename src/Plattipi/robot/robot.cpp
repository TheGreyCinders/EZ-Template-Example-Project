#include "plattipi/robot/robot.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/llemu.hpp"
#include "Plattipi/utils.hpp"  // IWYU pragma: keep
#include "pros/rtos.hpp"

//blue left
ASSET(BL1mogotoringstacks_txt);
ASSET(BL2ringstacktobackup_txt);
ASSET(BL3backuptocorner_txt);
ASSET(BL4cornertobackup2_txt);
ASSET(BL5backup2tocenter_txt);

//blue right
ASSET(BR1starttomogo_txt);
ASSET(BR2mogotoringstacks_txt);
ASSET(BR3ringstackstocorner_txt);

//red left
ASSET(RL1mogotoringstacks_txt);
ASSET(RL2ringstacktobackup_txt);
ASSET(RL3backuptocorner_txt);
ASSET(RL4cornertobackup2_txt);
ASSET(RL5backup2tocenter_txt);

//red right
ASSET(RR1starttomogo_txt);
ASSET(RR2mogotoringstacks_txt);
ASSET(RR3ringstackstocorner_txt);

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

    // colorSortingToggle = true;

    m_drive_train.llChassis.setPose(0, 0, 0);

    pros::Task colorSortTask {[=] (void) {
      colorSortAuto();
    }};

    pros::Task periodicTask {[=] (void) {
      periodic();
    }};
  }

  void Robot::autoBlueLeft() {
    m_alliance = 1; //blue
    colorSortingToggle = true;
    double cornerInX{62.5};
    double cornerInY{-62.5};
    double cornerOutX{54};
    double cornerOutY{-54};
    //set pose
    m_drive_train.llChassis.setPose(59, -13.25, 0);

    //start pose
    m_intake.intakeIntake();
    m_arm.toggleIn();

    //move to alliance stake
    m_drive_train.llChassis.moveToPoint(59, 0, 1250, {.maxSpeed = 50}, true);
    m_drive_train.llChassis.turnToHeading(270, 1250, {.maxSpeed = 65}, true);
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
    m_drive_train.llChassis.moveToPoint(57.25, 0, 1000, {.maxSpeed = 65}, true);
    m_drive_train.llChassis.waitUntilDone();
    m_drive_train.llChassis.turnToHeading(90, 2000, {.maxSpeed = 40}, true);
    pros::delay(375);
    m_drive_train.llChassis.moveToPoint(46, 0, 1000, {.forwards = false, .maxSpeed = 65}, true);
    m_drive_train.llChassis.waitUntil(10);
    m_mogo_mech.toggleGrabbed();

    m_drive_train.llChassis.waitUntilDone();

    //grab ring stacks
    sortingControl = true;

    pros::delay(250);

    m_drive_train.llChassis.follow(BL1mogotoringstacks_txt, 10, 5000, true, true);
    m_drive_train.llChassis.waitUntilDone();

    pros::delay(1000);

    // //go to corner
    m_drive_train.llChassis.follow(BL2ringstacktobackup_txt, 10, 4000, false, true);
    m_drive_train.llChassis.follow(BL3backuptocorner_txt, 10, 1500, true, true);
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
    // m_drive_train.llChassis.follow(BL4cornertobackup2_txt, 10, 2000, false, true);
    m_drive_train.llChassis.moveToPoint(24, -50, 2000, {.maxSpeed = 100}, true);
    m_drive_train.llChassis.turnToHeading(350, 2000, {.maxSpeed = 100}, true);
    m_drive_train.llChassis.waitUntilDone();
    intakeStop();
    conveyorStop();
    m_arm.toggleOut();
    // m_arm.toggleOut();
    m_drive_train.llChassis.moveToPoint(20, 20, 2000, {.maxSpeed = 65}, true);
    // m_drive_train.llChassis.follow(BL5backup2tocenter_txt, 10, 2000, true, true);
  }


  void Robot::autoRedLeft() {
    m_alliance = 2; //red
    colorSortingToggle = true;
    double cornerInX{-64};
    double cornerInY{59.5};
    double cornerOutX{-54};
    double cornerOutY{54};
    //set pose
    m_drive_train.llChassis.setPose(-59, 13.25, 180);

    //start pose
    m_intake.intakeIntake();
    m_arm.toggleIn();

    //move to alliance stake
    m_drive_train.llChassis.moveToPoint(-59, 0, 1250, {.maxSpeed = 50}, true);
    m_drive_train.llChassis.turnToHeading(90, 1250, {.maxSpeed = 65}, true);
    m_intake.intakeStop();
    m_drive_train.llChassis.moveToPoint(-63, 0, 750, {.maxSpeed = 65}, true);

    m_drive_train.llChassis.waitUntilDone();

    //score on stake
    intakeIntake();
    conveyorIntake();
    pros::delay(650);
    intakeStop();
    conveyorStop();

    m_drive_train.llChassis.waitUntilDone();

    //grab mogo
    m_drive_train.llChassis.moveToPoint(-57, 0, 1000, {.maxSpeed = 65}, true);
    m_drive_train.llChassis.waitUntilDone();
    m_drive_train.llChassis.turnToHeading(-90, 2000, {.maxSpeed = 60}, true);
    pros::delay(375);
    m_drive_train.llChassis.moveToPoint(-46, 0, 1000, {.forwards = false, .maxSpeed = 65}, true);
    m_drive_train.llChassis.waitUntil(10);
    m_mogo_mech.toggleGrabbed();

    m_drive_train.llChassis.waitUntilDone();

    //grab ring stacks
    sortingControl = true;

    pros::delay(250);

    m_drive_train.llChassis.follow(RL1mogotoringstacks_txt, 10, 5000, true, true);
    m_drive_train.llChassis.waitUntilDone();

    pros::delay(1000);

    // //go to corner
    m_drive_train.llChassis.follow(RL2ringstacktobackup_txt, 10, 4000, false, true);
    m_drive_train.llChassis.follow(RL3backuptocorner_txt, 10, 1500, true, true);
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
    m_drive_train.llChassis.follow(RL4cornertobackup2_txt, 10, 2000, false, true);
    m_drive_train.llChassis.waitUntilDone();
    intakeStop();
    conveyorStop();
    armToggleOut();
    // armToggleOut();
    m_drive_train.llChassis.follow(RL5backup2tocenter_txt, 10, 2000, true, true);
  }

  void Robot::autoBlueRight() {
    m_alliance = 1; //blue
    colorSortingToggle = true;
    double cornerInX{59.5};
    double cornerInY{59};
    double cornerOutX{55.5};
    double cornerOutY{52.5};

   m_drive_train.llChassis.setPose(51, 14, 90);

    m_arm.toggleIn();
    m_intake.intakeOuttake();

    //grab mogo
    m_drive_train.llChassis.follow(BR1starttomogo_txt, 11, 2000, false, true);
    // m_drive_train.llChassis.waitUntil(44.5);
    m_drive_train.llChassis.waitUntilDone();
    mogoToggle();
    m_drive_train.llChassis.waitUntilDone();
    intakeIntake();
    sortingControl = true;

    pros::delay(75);

    // //intake ring stacks
    m_drive_train.llChassis.follow(BR2mogotoringstacks_txt, 10, 6000, true, true);
    m_drive_train.llChassis.waitUntil(20);
    conveyorIntake();
    m_drive_train.llChassis.waitUntilDone();

    // //corner RNG
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
    m_drive_train.llChassis.moveToPoint(cornerOutX, cornerOutY,800, {.forwards = false, .maxSpeed = 50}, true);
    m_drive_train.llChassis.moveToPoint(cornerInX, cornerInY, 800, {.maxSpeed = 50}, true);
    pros::delay(9000);
    // //drop in corner
    m_drive_train.llChassis.follow(BR3ringstackstocorner_txt, 10, 3250, false, true);
    // m_drive_train.llChassis.moveToPoint(45, 45, 1500, {.forwards = false, .maxSpeed = 75}, true);
    // m_drive_train.llChassis.turnToHeading(225, 1500, {.maxSpeed = 75}, true);
    // m_drive_train.llChassis.moveToPoint(cornerOutX, 55.5, 1500, {.forwards = false, .maxSpeed = 75}, true);
    m_drive_train.llChassis.waitUntilDone();
    intakeStop();
    conveyorStop();
    mogoToggle();
    m_drive_train.llChassis.moveToPoint(45, -45, 1500, {.maxSpeed = 75}, true);
  }

  void Robot::autoRedRight() {
    m_alliance = 2; //red
    colorSortingToggle = true;
    double cornerInX{-59.5};
    double cornerInY{-60};
    double cornerOutX{-55.5};
    double cornerOutY{-52.5};

    m_drive_train.llChassis.setPose(-51, -14, 270);

    // m_drive_train.llChassis.moveToPoint(-20, -16, 5000);

    m_arm.toggleIn();
    intakeOuttake();

    //grab mogo
    m_drive_train.llChassis.follow(RR1starttomogo_txt, 12, 2000, false, true);
    // m_drive_train.llChassis.waitUntil(44.5);
    m_drive_train.llChassis.waitUntilDone();
    mogoToggle();
    m_drive_train.llChassis.waitUntilDone();
    intakeIntake();
    sortingControl = true;

    pros::delay(75);

    // //intake ring stacks
    m_drive_train.llChassis.follow(RR2mogotoringstacks_txt, 10, 6000, true, true);
    m_drive_train.llChassis.waitUntil(20);
    conveyorIntake();
    m_drive_train.llChassis.waitUntilDone();

    // //corner RNG
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

    // //drop in corner
    // m_drive_train.llChassis.follow(RR3ringstackstocorner_txt, 10, 6000, false, true);
    m_drive_train.llChassis.moveToPoint(-45, -45, 1500, {.forwards = false, .maxSpeed = 75}, true);
    m_drive_train.llChassis.turnToHeading(45, 1500, {.maxSpeed = 75}, true);
    m_drive_train.llChassis.moveToPoint(cornerOutX, -55.5, 1500, {.forwards = false, .maxSpeed = 75}, true);
    m_drive_train.llChassis.waitUntilDone();
    intakeStop();
    conveyorStop();
    mogoToggle();
    m_drive_train.llChassis.moveToPoint(-45, -45, 1500, {.maxSpeed = 75}, true);
  }

  void Robot::autoSkillsBlue() {
    m_alliance = 2;
    colorSortingToggle = false;

    m_drive_train.llChassis.setPose(-60, 0, 90);

    m_intake.intakeIntake();
    m_arm.toggleIn();

    m_drive_train.llChassis.moveToPoint(-54, 0, 2000, {.maxSpeed =75}, true);
    pros::delay(500);

    m_drive_train.llChassis.moveToPoint(-64, 0, 2000, {.forwards = false, .maxSpeed = 60}, true);
    m_drive_train.llChassis.waitUntilDone();

    intakeIntake();
    conveyorIntake();

    pros::delay(3000);
  }


  bool mogoGrabbed = false;

  void Robot::periodic() {
    while (true) {
      //drive
      m_intake.periodic();
      m_conveyor.periodic();
      m_arm.periodic();
      //mogo mech
      
      pros::lcd::print(1, "Blue: %f", m_intake.getBlue());
      pros::lcd::print(2, "Red: %f", m_intake.getRed());
      pros::lcd::print(3, "Color: %i", m_intake.getColor());
      pros::lcd::print(4, "X: %f", m_drive_train.llChassis.getPose().x);
      pros::lcd::print(5, "Y: %f", m_drive_train.llChassis.getPose().y);
      pros::lcd::print(6, "Theta: %f", m_drive_train.llChassis.getPose().theta);
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
          pros::lcd::print(7, "test");
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

  void Robot::test2() {
    m_conveyor.retract();
  }
}  // namespace robot
}  // namespace plattipi