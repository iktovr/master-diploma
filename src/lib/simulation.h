#pragma once

#include "agent.h"
#include "dispatch.h"
#include "graph.h"
#include "visualizer.h"

#include <optional>
#include <vector>

/* 
число заказов
средняя скорость
среднее время шага симуляции
среднее время ожидания
среднее время симуляции
*/

struct Simulation {
    double speed;
    Agents agents;
    Graph graph;
    Dispatch dispatch;
    std::optional<Visualizer> vis;

    Simulation(const double speed_, const Agents& agents_, const Graph& graph_, const std::optional<Visualizer> vis_ = std::nullopt) :
        speed(speed_), agents(agents_), graph(graph_), dispatch(graph, agents), vis(vis_) {
        if (vis) {
            vis->DrawGraph(graph);
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
    void Visualize();
    void Step(const double dt);
};
