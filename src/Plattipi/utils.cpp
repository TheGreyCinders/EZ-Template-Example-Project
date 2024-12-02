#include "Plattipi/utils.hpp" // IWYU pragma: keep

bool pointWithinRange(double currentX, double currentY, double targetX, double targetY, double distance) {
    if ((sqrt(pow(targetX-currentX, 2)+pow(targetY-currentY, 2))) < distance) {
        return true;
    } else {
        return false;
    }
}
