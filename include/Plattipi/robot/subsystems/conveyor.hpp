#pragma once

#include "pros/motor_group.hpp"
#include "pros/adi.hpp"
namespace plattipi {
namespace robot {
namespace subsystems {
    class Conveyor {
        private:
        std::vector<pros::Motor> m_conveyor_motors;
        pros::adi::DigitalOut m_piston;

        static constexpr double CONVEYOR_VELOCITY{600};
        double currentVelocity{0};

        public:
        Conveyor();
        Conveyor(std::vector<pros::Motor>& conveyor_motors, pros::adi::DigitalOut piston);

        void initialize();
        void intake();
        void outtake();
        void extend();
        void retract();
        void stop();
        
        void periodic();

    };

}
}
}