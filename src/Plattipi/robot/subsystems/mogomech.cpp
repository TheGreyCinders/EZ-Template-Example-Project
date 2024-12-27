#include "plattipi/robot/subsystems/mogomech.hpp"

namespace plattipi {
namespace robot {
namespace subsystems {
    MogoMech::MogoMech(pros::adi::DigitalOut piston, pros::adi::DigitalOut pistonTest) : m_piston{piston}, m_pistonTest{pistonTest} {}

    void MogoMech::toggleGrabbed() {
        m_piston.set_value(mogoGrabbed);
        mogoGrabbed = !mogoGrabbed;
    }

    void MogoMech::togglePiston() {
        m_pistonTest.set_value(pistonGrabbed);
        pistonGrabbed = !pistonGrabbed;
    }
    
}  // namespace subsystems
}  // namespace robot
}  // namespace plattipi