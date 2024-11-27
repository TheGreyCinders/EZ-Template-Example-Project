#include "plattipi/robot/subsystems/mogomech.hpp"

namespace plattipi {
namespace robot {
namespace subsystems {
    MogoMech::MogoMech(pros::adi::DigitalOut piston) : m_piston{piston} {}

    void MogoMech::toggleGrabbed() {
        m_piston.set_value(mogoGrabbed);
        mogoGrabbed = !mogoGrabbed;
    }
    
}  // namespace subsystems
}  // namespace robot
}  // namespace plattipi