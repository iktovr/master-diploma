#include "simulation.h"

#include <chrono>
#include <cmath>
#include <functional>
#include <queue>
#include <utility>

#include "logging.h"

namespace {

constexpr double kTimeEps = 1e-9;

struct Event {
    double time;
    double period;
    int    priority;
    std::function<void(double)> action;

    void Evaluate() {
        action(time);
        time += period;
    }
};

bool operator>(const Event& a, const Event& b) {
    if (std::abs(a.time - b.time) > kTimeEps) {
        return a.time > b.time;
    }
    return a.priority > b.priority;
}

}  // namespace

void Simulation::Step(const double t, const double dt) {
    semaphores.Step(t, agents);
    dispatch.Step(agents);
    for (auto& agent : agents) {
        agent.Move(dt, speed);
    }
}

void Simulation::Visualize() {
    if (!vis) {
        return;
    }
    vis->ClearFrame();
    for (const auto& agent : agents) {
        vis->DrawAgent(agent);
    }
    vis->SaveFrame();
}

void Simulation::Simulate(const double duration, const double step, const double vis_step) {
    std::priority_queue<Event, std::vector<Event>, std::greater<Event>> events;

    events.push(Event{step, step, 0,
        [this, step](double t) { Step(t, step); }});

    if (vis && vis_step > 0) {
        events.push(Event{0.0, vis_step, 1,
            [this](double) { Visualize(); }});
    }

    const double progress_step = duration / 10.0;
    if (progress_step > 0) {
        events.push(Event{0.0, progress_step, 2,
            [duration](double t) {
                LOG_INFO("Simulation progress: {:.1f}% (t = {:.2f} / {:.2f})",
                        100.0 * t / duration, t, duration);
                }});
    }

    auto start = std::chrono::high_resolution_clock::now();

    while (!events.empty()) {
        Event event = std::move(const_cast<Event&>(events.top()));
        events.pop();
        if (event.time > duration + kTimeEps) {
            break;
        }
        event.Evaluate();
        events.push(std::move(event));
    }

    auto end = std::chrono::high_resolution_clock::now();
    double real_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() / 1e9;
    LOG_INFO("Simulation virtual duration: {}", duration);
    LOG_INFO("Simulation real duration: {}", real_duration);
    LOG_INFO("Simulation speed: {} s(v)/s", duration / real_duration);
}
