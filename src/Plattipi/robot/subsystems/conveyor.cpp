#include "plattipi/robot/subsystems/conveyor.hpp"

namespace plattipi {
namespace robot {
namespace subsystems {
    Conveyor::Conveyor(std::vector<pros::Motor>& conveyor_motors, pros::adi::DigitalOut piston) : m_conveyor_motors{conveyor_motors}, m_piston{piston} {}

    void Conveyor::initialize() {
        for (auto motor : m_conveyor_motors) {
            motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        }
        m_piston.set_value(false);
    }

    void Conveyor::intake() {
        currentVelocity = CONVEYOR_VELOCITY;
    }

    void Conveyor::outtake() {
        currentVelocity = -CONVEYOR_VELOCITY;
    }

    void Conveyor::stop() {
        currentVelocity = 0;
    }

    void Conveyor::extend() {
        m_piston.set_value(true);
    }

    void Conveyor::retract() {
        m_piston.set_value(false);
    }

    void Conveyor::periodic() {
        for (auto motor : m_conveyor_motors) {
            motor.move_velocity(currentVelocity);
        }
    }

}  // namespace subsystems
}  // namespace robot
}  // namespace plattipi