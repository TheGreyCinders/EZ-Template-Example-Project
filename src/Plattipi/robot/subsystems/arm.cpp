#include "plattipi/robot/subsystems/arm.hpp"

namespace plattipi {
namespace robot {
namespace subsystems {
    Arm::Arm(pros::Motor armLeft, pros::Motor armRight, pros::Motor extension) : m_armLeft{armLeft}, m_armRight{armRight}, m_extension{extension} {}
    
    void Arm::goLoad() {
        state = ArmStates::LOADING;
    }

    void Arm::goStore() {
        state = ArmStates::STORING;
    } 

    void Arm::goHold() {
        state = ArmStates::HOLDING;
    }

    void Arm::toggleOut() {
        pastState = state;
        armMoving = true;
        rotationMoving = true;
        extensionMoving = true;
        if (state == ArmStates::LOADING) {
            goHold();
        } else if (state == ArmStates::HOLDING || state == ArmStates::STORING) {
            goLoad();
        }
    }

    void Arm::toggleIn() {
        pastState = state;
        armMoving = true;
        rotationMoving = true;
        extensionMoving = true;
        if (state == ArmStates::LOADING) {
            goStore();
        } else if (state == ArmStates::HOLDING || state == ArmStates::STORING) {
            goLoad();
        }
    }

    void Arm::updateArm() {
        //holding macros
        if (state == ArmStates::HOLDING) {
            //loading-holding macro
            if (pastState == ArmStates::LOADING) {
            if (rotationMoving == true) {
                m_armLeft.move_absolute(ARM_UP, -ARM_ROTATE_SPEED);
                m_armRight.move_absolute(ARM_UP, -ARM_ROTATE_SPEED);
                rotationMoving = false;
            }
            if (extensionMoving == true && m_armLeft.get_position() > 125) {
                m_extension.move_absolute(EXT_OUT, -EXT_SPEED);
                extensionMoving = false;
            }
            }
            //storing-holding macro
            if (pastState == ArmStates::HOLDING) {
            //case is impossible
            }
        } 

        //loading macros
        else if (state == ArmStates::LOADING) {
            //holding-loading macro
            if (pastState == ArmStates::HOLDING) {
            if (extensionMoving == true) {
                m_extension.move_absolute(EXT_IN, EXT_SPEED);
                extensionMoving = false;
            }
            if (rotationMoving == true && m_extension.get_position() < 200) {
                m_armLeft.move_absolute(ARM_DOWN, ARM_ROTATE_SPEED);
                m_armRight.move_absolute(ARM_DOWN, ARM_ROTATE_SPEED);
                rotationMoving = false;
            }
            }
            //storing-loading macro
            if (pastState == ArmStates::STORING) {
            if (extensionMoving == true) {
                m_extension.move_absolute(EXT_IN, EXT_SPEED);
                extensionMoving = false;
            }
            rotationMoving = true;
            }
        }

        else if (state == ArmStates::STORING) {
            //storing-holding macro
            if (pastState == ArmStates::HOLDING) {
            if (extensionMoving == true) {
                m_extension.move_absolute(EXT_STORE, EXT_SPEED);
                extensionMoving = false;
            }
            if (rotationMoving == true && m_extension.get_position() < 200) {
                m_armLeft.move_absolute(ARM_DOWN, ARM_ROTATE_SPEED);
                m_armRight.move_absolute(ARM_DOWN, ARM_ROTATE_SPEED);
                rotationMoving = false;
            }
            }
            //loading-storing macroar
            if (pastState == ArmStates::LOADING) {
            if (extensionMoving == true) {
                m_extension.move_absolute(EXT_STORE, EXT_SPEED);
                extensionMoving = false;
            }
            rotationMoving = true;
            }
        }
        if (!extensionMoving && !rotationMoving) {
            armMoving = false;
        }
    }

    void Arm::periodic() {
        if (armMoving=true) {
            Arm::updateArm();
        }
    }

    
}  // namespace subsystems
}  // namespace robot
}  // namespace plattipi