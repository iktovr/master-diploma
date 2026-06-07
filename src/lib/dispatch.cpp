#include "dispatch.h"

#include <cmath>
#include <random>
#include <vector>

#include "agent.h"
#include "statistics.h"
#include "geometry.h"

namespace lib {

void Dispatch::Step(double t, Agents& agents) {
    for (size_t i = 0; i < agents.size(); ++i) {
        auto& agent = agents[i];

        if (order_start_time[i] >= 0) {
            traveled_length[i] += bg::distance(last_pos[i], agent.pos);
        }
        last_pos[i] = agent.pos;

        if (agent.state != Agent::idle) {
            continue;
        }

        if (order_start_time[i] >= 0) {
            const double elapsed = t - order_start_time[i];
            if (elapsed > 0.0) {
                Statistics::Get().speed.Add(traveled_length[i] / elapsed);
                Statistics::Get().order_time.Add(elapsed);
            }
            order_start_time[i] = -1;
            traveled_length[i] = 0.0;
        }

        if (current_order[i] == -1) {
            int order = NewOrder(agent.base);
            current_order[i] = order;
            auto [route, vids] = router->GetRouteWithVertices(
                agent.base, order, t, static_cast<int>(i));
            // hack for ccbs
            if (agent.route_follower.segment_schedule_t.empty()) {
                agent.SetRoute(route, vids, t);
            }
            traveled_length[i] = 0.0;
            last_pos[i] = agent.pos;
            order_start_time[i] = t;
        } else {
            auto [route, vids] = router->GetRouteWithVertices(
                current_order[i], agent.base, t, static_cast<int>(i));
            if (agent.route_follower.segment_schedule_t.empty()) {
                agent.SetRoute(route, vids, t);
            }
            current_order[i] = -1;
            Statistics::Get().orders_count++;
            traveled_length[i] = 0.0;
            last_pos[i] = agent.pos;
            order_start_time[i] = t;
        }
    }
}

int Dispatch::NewOrder(int base_id) const {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    auto it = delivery_points_by_base.find(base_id);
    const std::vector<int>& pool =
        (it != delivery_points_by_base.end() && !it->second.empty())
            ? it->second
            : delivery_points;
    std::uniform_int_distribution<int> dist(0, pool.size() - 1);
    return pool[dist(gen)];
}

void Dispatch::AssignBasePoints(Agents& agents) const {
    for (size_t i = 0; i < agents.size(); ++i) {
        size_t base = i % base_points.size();
        agents[i].base = base_points[base];
        agents[i].pos = graph->vertices[base_points[base]].pos;
    }
}

}
