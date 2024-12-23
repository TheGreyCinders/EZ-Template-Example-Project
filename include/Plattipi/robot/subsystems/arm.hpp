#pragma once

#include "pros/motors.hpp"
namespace plattipi {
namespace robot {
namespace subsystems {
    class Arm {
        private:
        pros::Motor m_armLeft;
        pros::Motor m_armRight;
        pros::Motor m_extension;

        bool armMoving, rotationMoving, extensionMoving;

        static constexpr double ARM_DOWN{0};
        static constexpr double ARM_UP{220};
        static constexpr double ARM_ROTATE_SPEED{85};

        static constexpr double EXT_IN{-5};
        static constexpr double EXT_STORE{210};
        static constexpr double EXT_OUT{680};
        static constexpr double EXT_SPEED{200};

        enum class ArmStates {
            HOLDING,
            LOADING,
            STORING
        };
        ArmStates state{ArmStates::LOADING};
        ArmStates pastState{ArmStates::LOADING};

        void updateArm();

        public:
        Arm();
        Arm(pros::Motor armLeft, pros::Motor armRight, pros::Motor extension);

        void initialize();
        void goLoad();
        void goStore();
        void goHold();
        void toggleOut();
        void toggleIn();
        void periodic();
        void getArmState();

    };
}
}
}