#pragma once

#include "geometry.h"

struct Agent {
    enum State {
        idle,
        move,
        wait
    };

    Point pos;
    State state = idle;

    Agent() = default;

    Agent(double x, double y) : pos(x, y) {}

    void Move(double dt, double speed);
};