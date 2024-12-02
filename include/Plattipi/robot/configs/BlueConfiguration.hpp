#pragma once
#include <cstdint>

namespace plattipi {
namespace robot {
namespace configs {

    class BlueConfiguration {
        private:
            

        public:
            BlueConfiguration();
            int test2 = 2;
            static constexpr int8_t LEFT_1_PORT = -1;
            static constexpr int8_t LEFT_2_PORT = 2;
            static constexpr int8_t LEFT_3_PORT = -3;
            static constexpr int8_t LEFT_4_PORT = 4;

            static constexpr int8_t RIGHT_4_PORT = -7;
            static constexpr int8_t RIGHT_3_PORT = 8;
            static constexpr int8_t RIGHT_2_PORT = -9;
            static constexpr int8_t RIGHT_1_PORT = 10;
    };

}
}
}