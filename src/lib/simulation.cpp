#include "simulation.h"

#include <functional>
#include <queue>

struct Event {
    double time;
    double period;
    std::function<void()> action;

    Event& Evaluate() {
        time += period;
        action();
        return *this;
    }
};

bool operator>(const Event& a, const Event& b) {
    if (abs(a.time - b.time) > 1e-3) {
        return a.time > b.time;
    }
    return a.period > b.period;
}

void Simulation::Step(const double dt) {
    dispatch.Step(agents);
    for (auto& agent: agents) {
        agent.Move(dt, 2.2);
    }
}

void Simulation::Visualize() {
    if (!vis) {
        return;
    }
    vis->ClearFrame();
    for (const auto& agent: agents) {
        vis->DrawAgent(agent);
    }
    vis->SaveFrame();
}

void Simulation::Simulate(const double duration, const double step, const double vis_step) {
    std::priority_queue<Event, std::vector<Event>, std::greater<Event>> events;
    events.push(Event{0.0, step, [this, step](){ Step(step); }});
    if (vis && vis_step > 0) {
        events.push(Event{0.0, vis_step, [this]() { Visualize(); }});
    }

    while (!events.empty()) {
        auto event = events.top();
        if (event.time > duration) {
            break;
        }
        events.pop();
        events.push(event.Evaluate());
    }
}
