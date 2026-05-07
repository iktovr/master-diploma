#include "dispatch.h"

#include <cmath>
#include <random>
#include <vector>

#include "agent.h"
#include "graph.h"

void Dispatch::Step(Agents& agents) {
    for (size_t i = 0; i < agents.size(); ++i) {
        auto& agent = agents[i];
        if (agent.state != Agent::idle) {
            continue;
        }

        if (current_order[i] == -1) {
            int order = NewOrder();
            current_order[i] = order;
            agent.SetRoute(graph.GetRoute(agent.base, order));
        } else {
            agent.SetRoute(graph.GetRoute(current_order[i], agent.base));
            current_order[i] = -1;
        }
    }
}

int Dispatch::NewOrder() const {
    static std::mt19937 gen;
    std::uniform_int_distribution<int> dist(0, delivery_points.size() - 1);
    return delivery_points[dist(gen)];
}

void Dispatch::AssignBasePoints(Agents& agents) const {
    for (size_t i = 0; i < agents.size(); ++i) {
        size_t base = i % base_points.size();
        agents[i].base = base_points[base];
        agents[i].pos = graph.vertices[base_points[base]].pos;
    }
}
