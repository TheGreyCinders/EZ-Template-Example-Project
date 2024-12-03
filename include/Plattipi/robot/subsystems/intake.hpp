#pragma once

#include "pros/motors.hpp"
#include "pros/optical.hpp"
namespace plattipi {
namespace robot {
namespace subsystems {
    class Intake {
        private:
        pros::Motor m_intake_motor;
        // std::vector<pros::Motor> m_conveyor_motors;
        pros::Optical m_color_sensor;

        static constexpr double INTAKE_VELOCITY{600};
        double currentIntakeVelocity{0};
        // double currentConveyorVelocity{0};
        
        double detection{0};
        int detectionColor{0};

        static constexpr double REVERSE_TIME{100};
        static constexpr double CONTINUE_TIME{200};
        static constexpr double END_TIME{300};

        public:
        Intake();
        Intake(pros::Motor intake_motor, pros::Optical color_sensor);

        void initialize();

        //intake
        void intakeIntake();
        void intakeOuttake();
        void intakeStop();

        void intakeSort();

        int detectColor(double hue);
        int getColor();
        double getHue();
        
        void periodic();

    };

}
}
}