#include "plattipi/robot/subsystems/intake.hpp"

namespace plattipi {
namespace robot {
namespace subsystems {
    Intake::Intake(pros::Motor intake_motor) : m_intake_motor{intake_motor} {}

    void Intake::initialize() {
        m_intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    }

    void Intake::intake() {
        currentVelocity = INTAKE_VELOCITY;
    }

    void Intake::outtake() {
        currentVelocity = -INTAKE_VELOCITY;
    }

    void Intake::stop() {
        currentVelocity = 0;
    }

    void Intake::periodic() {
        m_intake_motor.move_velocity(currentVelocity);
    }

}  // namespace subsystems
}  // namespace robot
}  // namespace plattipi