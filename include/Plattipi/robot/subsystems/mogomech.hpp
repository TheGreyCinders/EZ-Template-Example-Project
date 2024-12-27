#pragma once

#include "pros/adi.hpp"
namespace plattipi {
namespace robot {
namespace subsystems {
    class MogoMech {
        private:
            bool mogoGrabbed{true};
            bool pistonGrabbed{true};
            pros::adi::DigitalOut m_piston;
            pros::adi::DigitalOut m_pistonTest;
        
        public:
            MogoMech();
            MogoMech(pros::adi::DigitalOut piston, pros::adi::DigitalOut pistonTest);

            void toggleGrabbed();
            void togglePiston();
    };
}
}
}