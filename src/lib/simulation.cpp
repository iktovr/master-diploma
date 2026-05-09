#include "simulation.h"

#include <chrono>
#include <functional>
#include <iostream>
#include <queue>

struct Event {
    double time;
    double period;
    std::function<void(double)> action;

    Event& Evaluate() {
        action(time);
        time += period;
        return *this;
    }
};

bool operator>(const Event& a, const Event& b) {
    if (abs(a.time - b.time) > 1e-3) {
        return a.time > b.time;
    }
    return a.period > b.period;
}

void Simulation::Step(const double t, const double dt) {
    semaphores.Step(t, agents);
    dispatch.Step(agents);
    for (auto& agent: agents) {
        agent.Move(dt, speed);
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
    events.push(Event{0.0, step, [this, step](double t){ Step(t, step); }});
    if (vis && vis_step > 0) {
        events.push(Event{0.0, vis_step, [this](double) { Visualize(); }});
    }

    auto start = std::chrono::high_resolution_clock::now();

    while (!events.empty()) {
        auto event = events.top();
        if (event.time > duration) {
            break;
        }
        events.pop();
        events.push(event.Evaluate());
    }

    auto end = std::chrono::high_resolution_clock::now();
    double real_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() / 1e9;
    std::cerr << "Simulation virtual duration: " << duration << '\n'
        << "Simulation real duration: " << real_duration << '\n'
        << "Simulation speed: " << duration / real_duration << " s(v)/s" << std::endl;
}
