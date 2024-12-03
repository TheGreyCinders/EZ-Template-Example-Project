#include "plattipi/robot/subsystems/intake.hpp"

namespace plattipi {
namespace robot {
namespace subsystems {
    Intake::Intake(pros::Motor intake_motor, pros::Optical color_sensor) : m_intake_motor{intake_motor}, m_color_sensor(color_sensor) {}

    void Intake::initialize() {
        m_intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    }

    //intake
    void Intake::intakeIntake() {
        // sorting = false;
        currentIntakeVelocity = INTAKE_VELOCITY;
    }

    void Intake::intakeOuttake() {
        // sorting = false;
        currentIntakeVelocity = -INTAKE_VELOCITY;
    }

    void Intake::intakeStop() {
        // sorting = false;
        currentIntakeVelocity = 0;
    }

    //conveyor
    // void Intake::conveyorIntake() {
    //     currentConveyorVelocity = INTAKE_VELOCITY;
    // }

    // void Intake::conveyorOuttake() {
    //     currentConveyorVelocity = -INTAKE_VELOCITY;
    // }

    // void Intake::conveyorStop() {
    //     currentConveyorVelocity = 0;
    // }

    //misc
    void Intake::intakeSort() {
        currentIntakeVelocity = INTAKE_VELOCITY;
        // sorting = true;
    }

    int Intake::detectColor(double hue) {
        if (hue < 20 || hue > 355) {
            return 1;
        } else if (hue > 190 && hue < 230) {
            return 2;
        } else {
            return 0;
        }
    }

    int Intake::getColor() {
        return detectionColor;
    }

    double Intake::getHue() {
        return detection;
    }

    void Intake::periodic() {
        // if (!sorting) {
        //     m_intake_motor.move_velocity(currentIntakeVelocity);
        //     m_conveyor_motors[0].move_velocity(currentConveyorVelocity);
        //     m_conveyor_motors[1].move_velocity(currentConveyorVelocity);
        // } else {
        //     if (!detected) {
        //         detection = m_color_sensor.get_hue();
        //         if (detectionColor == 1) {
        //             detected = true;
        //             detectTime = pros::millis();
        //         } else {
        //             m_intake_motor.move_velocity(currentIntakeVelocity);
        //             for (auto motor : m_conveyor_motors) {
        //                 motor.move_velocity(currentConveyorVelocity);
        //             }
        //         }
        //     } else {
        //         if ((detectTime - END_TIME) > pros::millis()) {
        //             detection = false;
        //         } else if ((detectTime - CONTINUE_TIME) > pros::millis()) {
        //             m_intake_motor.move_velocity(currentIntakeVelocity);
        //             for (auto motor : m_conveyor_motors) {
        //                 motor.move_velocity(currentConveyorVelocity);
        //             }
        //         } else if ((detectTime - REVERSE_TIME) > pros::millis()) {
        //             m_intake_motor.move_velocity(-currentIntakeVelocity);
        //             for (auto motor : m_conveyor_motors) {
        //                 motor.move_velocity(-currentConveyorVelocity);
        //             }
        //         }
        //     }
        // }

        detection = m_color_sensor.get_hue();
        detectionColor = detectColor(detection);
        m_intake_motor.move_velocity(currentIntakeVelocity);
    }

}  // namespace subsystems
}  // namespace robot
}  // namespace plattipi