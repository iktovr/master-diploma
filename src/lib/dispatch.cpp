#include "dispatch.h"

#include <cmath>
#include <random>
#include <vector>

#include "agent.h"
#include "statistics.h"
#include "geometry.h"

void Dispatch::Step(double t, Agents& agents) {
    for (size_t i = 0; i < agents.size(); ++i) {
        auto& agent = agents[i];
        if (agent.state != Agent::idle) {
            continue;
        }

        if (order_start_time[i] > 0) {
            Statistics::Get().speed.Add(route_length[i] / (t - order_start_time[i]));
            order_start_time[i] = -1;
            route_length[i] = -1;
        }

        if (current_order[i] == -1) {
            int order = NewOrder();
            current_order[i] = order;
            agent.SetRoute(router->GetRoute(agent.base, order));
            route_length[i] = bg::length(agent.Route());
            order_start_time[i] = t;
        } else {
            agent.SetRoute(router->GetRoute(current_order[i], agent.base));
            current_order[i] = -1;
            Statistics::Get().orders_count++;
            route_length[i] = bg::length(agent.Route());
            order_start_time[i] = t;
        }
    }
}

int Dispatch::NewOrder() const {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dist(0, delivery_points.size() - 1);
    return delivery_points[dist(gen)];
}

void Dispatch::AssignBasePoints(Agents& agents) const {
    for (size_t i = 0; i < agents.size(); ++i) {
        size_t base = i % base_points.size();
        agents[i].base = base_points[base];
        agents[i].pos = graph->vertices[base_points[base]].pos;
    }
}
