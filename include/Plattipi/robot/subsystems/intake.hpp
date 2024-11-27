#pragma once

#include "pros/motor_group.hpp"
namespace plattipi {
namespace robot {
namespace subsystems {
    class Intake {
        private:
        pros::Motor m_intake_motor;

        static constexpr double INTAKE_VELOCITY{600};
        double currentVelocity{0};

        public:
        Intake();
        Intake(pros::Motor intake_motor);

        void initialize();
        void intake();
        void outtake();
        void stop();
        
        void periodic();

    };

}
}
}