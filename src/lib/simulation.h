#pragma once

#include <memory>
#include <optional>
#include <vector>

#include "agent.h"
#include "dispatch.h"
#include "graph.h"
#include "router.h"
#include "semaphore.h"
#include "visualizer.h"

struct Simulation {
    double speed;
    Agents agents;
    std::shared_ptr<const Graph> graph;
    Dispatch dispatch;
    SemaphoreManager semaphores;
    std::optional<Visualizer> vis;

    Simulation(
        const double speed_,
        const Agents& agents_,
        std::shared_ptr<const Graph> graph_,
        const std::shared_ptr<const IRouter> router_,
        const std::optional<Visualizer> vis_ = std::nullopt)
        : speed(speed_)
        , agents(agents_)
        , graph(std::move(graph_))
        , dispatch(graph, router_, agents)
        , semaphores(graph)
        , vis(vis_)
    {
        if (vis) {
            vis->DrawGraph(*graph);
            vis->SavePersistentPart();
        }
        dispatch.AssignBasePoints(agents);
    }

    void AddAgent(const Agent& agent) {
        agents.push_back(agent);
    }

    template <class... Args>
    void AddAgent(Args... args) {
        agents.emplace_back(args...);
    }

    void Simulate(const double duration, const double step, const double vis_step = -1);
    void Visualize(const double t);
    void Step(const double t, const double dt);
};
