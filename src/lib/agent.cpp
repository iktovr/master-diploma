#include "agent.h"

#include <cassert>
#include <cmath>

#include "geometry.h"
#include "statistics.h"

void RouteFollower::SetRoute(const Linestring& new_route) {
    assert(!new_route.empty());
    route = new_route;
    incoming_route = route;
    length = bg::length(route);
    x = 0;
    pos = route.front();
    vertex_ids.clear();
    cumulative_len.clear();
    segment_idx = 0;
    segment_t_enter = 0.0;
}

void RouteFollower::SetRoute(const Linestring& new_route,
                             const std::vector<int>& new_vertex_ids,
                             double t_now) {
    SetRoute(new_route);
    // vertex_ids must align 1-to-1 with route points to enable per-edge stats.
    assert(new_vertex_ids.size() == new_route.size());
    vertex_ids = new_vertex_ids;
    cumulative_len.assign(new_route.size(), 0.0);
    for (std::size_t i = 1; i < new_route.size(); ++i) {
        cumulative_len[i] = cumulative_len[i - 1]
            + bg::distance(new_route[i - 1], new_route[i]);
    }
    segment_idx = 0;
    segment_t_enter = t_now;
}

Point RouteFollower::Move(const double dx) {
    x += dx;
    if (x > length) {
        x = length;
    }
    if (incoming_route.size() > 2 && bg::distance(incoming_route[0], incoming_route[1]) < dx) {
        incoming_route.erase(incoming_route.begin());
    }
    bg::line_interpolate(route, x, pos);
    incoming_route[0] = pos;
    return pos;
}

Point RouteFollower::Move(const double dx, const double dt, const double t_now) {
    if (vertex_ids.empty() || dx <= 0.0 || dt <= 0.0) {
        return Move(dx);
    }

    const double x_before = x;
    const double t_before = t_now - dt;

    // Standard advance.
    x += dx;
    if (x > length) {
        x = length;
    }
    if (incoming_route.size() > 2 && bg::distance(incoming_route[0], incoming_route[1]) < dx) {
        incoming_route.erase(incoming_route.begin());
    }
    bg::line_interpolate(route, x, pos);
    incoming_route[0] = pos;

    const double dx_eff = x - x_before;
    if (dx_eff <= 0.0) {
        return pos;
    }
    const double dt_eff = dt * (dx_eff / dx);

    auto& stats = Statistics::Get().edges;
    while (segment_idx + 1 < vertex_ids.size()
            && x >= cumulative_len[segment_idx + 1]) {
        const double boundary = cumulative_len[segment_idx + 1];
        const double t_exit = t_before
            + dt_eff * (boundary - x_before) / dx_eff;
        const int u = vertex_ids[segment_idx];
        const int v = vertex_ids[segment_idx + 1];
        const double seg_len = boundary - cumulative_len[segment_idx];
        stats.Record(u, v, segment_t_enter, t_exit, seg_len, t_now);
        segment_t_enter = t_exit;
        ++segment_idx;
    }

    return pos;
}

void Agent::SetRoute(const Linestring& route) {
    route_follower.SetRoute(route);
    state = move;
}

void Agent::SetRoute(const Linestring& route,
                     const std::vector<int>& vertex_ids,
                     double t_now) {
    route_follower.SetRoute(route, vertex_ids, t_now);
    state = move;
}

void Agent::Move(const double dt, const double speed) {
    if (state != move) {
        return;
    }

    pos = route_follower.Move(speed * dt);
    if (route_follower.IsFinished()) {
        state = idle;
    }
}

void Agent::Move(const double t, const double dt, const double speed) {
    if (state != move) {
        return;
    }

    pos = route_follower.Move(speed * dt, dt, t);
    if (route_follower.IsFinished()) {
        state = idle;
    }
}
