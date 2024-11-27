#include "plattipi/robot/subsystems/conveyor.hpp"

namespace plattipi {
namespace robot {
namespace subsystems {
    Conveyor::Conveyor(std::vector<pros::Motor>& conveyor_motors) : m_conveyor_motors{conveyor_motors} {}

    void Conveyor::initialize() {
        for (auto motor : m_conveyor_motors) {
            motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        }
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

    void Conveyor::periodic() {
        for (auto motor : m_conveyor_motors) {
            motor.move_velocity(currentVelocity);
        }
    }

}  // namespace subsystems
}  // namespace robot
}  // namespace plattipi