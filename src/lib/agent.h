#pragma once

struct Agent {
    enum State {
        idle,
        move,
        wait
    };

    double x = 0;
    double y = 0;
    State state = idle;

    Agent() = default;

    Agent(double x, double y) : x(x), y(y) {}

    void Move(double dt, double speed);
};