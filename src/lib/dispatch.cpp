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

        // Accumulate ground-truth traveled distance for the active order
        // every tick, regardless of state. Using positional deltas (rather
        // than the planned route length at order-start) keeps the metric
        // correct across mid-trip CCBS replans, semaphore waits, and
        // reverse excursions.
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
            int order = NewOrder();
            current_order[i] = order;
            auto [route, vids] = router->GetRouteWithVertices(
                agent.base, order, t, static_cast<int>(i));
            // Some routers (e.g. CcbsRouter) install a scheduled route on
            // the caller agent themselves. Re-applying SetRoute(route,
            // vids, t) here would wipe |segment_schedule_t| and cause the
            // simulation to treat the agent as unscheduled — defeating
            // the schedule-aware follow-cap gate and the
            // ScheduledPosition clamp inside RouteFollower::Move().
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
