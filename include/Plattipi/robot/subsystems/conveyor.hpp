#pragma once

#include "pros/motor_group.hpp"
namespace plattipi {
namespace robot {
namespace subsystems {
    class Conveyor {
        private:
        std::vector<pros::Motor> m_conveyor_motors;

        static constexpr double CONVEYOR_VELOCITY{600};
        double currentVelocity{0};

        public:
        Conveyor();
        Conveyor(std::vector<pros::Motor>& conveyor_motors);

        void initialize();
        void intake();
        void outtake();
        void stop();
        
        void periodic();

    };

}
}
}