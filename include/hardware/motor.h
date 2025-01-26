#pragma once

namespace hardware {
enum class Gearset {
    RED = 100,
    GREEN = 200,
    BLUE = 600
};

class Motor {
    private:
        const Gearset gearset;
        const int port;

    public:
        Motor(int port, Gearset gearset);

        void move_voltage(int voltage);
        void move_velocity(int velocity);
};
};