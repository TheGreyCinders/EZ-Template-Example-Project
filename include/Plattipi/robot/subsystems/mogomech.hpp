#pragma once

#include "pros/adi.hpp"
namespace plattipi {
namespace robot {
namespace subsystems {
    class MogoMech {
        private:
            bool mogoGrabbed{true};
            pros::adi::DigitalOut m_piston;
        
        public:
            MogoMech();
            MogoMech(pros::adi::DigitalOut piston);

            void toggleGrabbed();
    };
}
}
}