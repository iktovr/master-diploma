#include "agent.h"

#include <algorithm>
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
    segment_entry_time.clear();
    max_segment_reached = 0;
}

void RouteFollower::SetRoute(const Linestring& new_route,
                             const std::vector<int>& new_vertex_ids,
                             double t_now) {
    SetRoute(new_route);
    assert(new_vertex_ids.size() == new_route.size());
    vertex_ids = new_vertex_ids;
    cumulative_len.assign(new_route.size(), 0.0);
    for (std::size_t i = 1; i < new_route.size(); ++i) {
        cumulative_len[i] = cumulative_len[i - 1]
            + bg::distance(new_route[i - 1], new_route[i]);
    }
    segment_idx = 0;
    segment_t_enter = t_now;
    segment_entry_time.assign(
        new_vertex_ids.empty() ? 0u : new_vertex_ids.size() - 1, 0.0);
    if (!segment_entry_time.empty()) {
        segment_entry_time[0] = t_now;
    }
    max_segment_reached = 0;
}

std::optional<std::pair<int, int>> RouteFollower::CurrentEdge() const {
    if (vertex_ids.empty() || segment_idx + 1 >= vertex_ids.size()) {
        return std::nullopt;
    }
    return std::make_pair(vertex_ids[segment_idx], vertex_ids[segment_idx + 1]);
}

double RouteFollower::DistanceAlongEdge() const {
    if (vertex_ids.empty() || segment_idx >= cumulative_len.size()) {
        return 0.0;
    }
    return x - cumulative_len[segment_idx];
}

double RouteFollower::CurrentEdgeLength() const {
    if (cumulative_len.size() < 2 || segment_idx + 1 >= cumulative_len.size()) {
        return 0.0;
    }
    return cumulative_len[segment_idx + 1] - cumulative_len[segment_idx];
}

void RouteFollower::RebuildIncomingRoute() {
    incoming_route.clear();
    incoming_route.push_back(pos);
    // Append the remaining route waypoints starting from the next vertex
    // after the current segment.
    for (std::size_t i = segment_idx + 1; i < route.size(); ++i) {
        incoming_route.push_back(route[i]);
    }
    if (incoming_route.size() < 2) {
        // Ensure incoming_route always has at least two points so existing
        // consumers (semaphore, visualizer) keep working.
        incoming_route.push_back(route.back());
    }
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

Point RouteFollower::Move(const double dx, const double dt, const double t_now,
                          const double max_dx) {
    const double dx_capped = dx < max_dx ? dx : max_dx;
    if (dx_capped <= 0.0) {
        return pos;
    }
    const double dt_scaled = dt * (dx_capped / dx);
    return Move(dx_capped, dt_scaled, t_now);
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

        // Only record the passage the first time we cross this boundary
        // going forward. If we've already crossed it before (and possibly
        // reversed back), the original entry/exit was recorded already.
        const std::size_t next_idx = segment_idx + 1;
        if (next_idx > max_segment_reached) {
            const double t_enter = segment_idx < segment_entry_time.size()
                ? segment_entry_time[segment_idx]
                : segment_t_enter;
            stats.Record(u, v, t_enter, t_exit, seg_len, t_now);
            max_segment_reached = next_idx;
            // Initialize the entry time of the new segment (if it exists).
            if (next_idx < segment_entry_time.size()) {
                segment_entry_time[next_idx] = t_exit;
            }
        }
        segment_t_enter = (next_idx < segment_entry_time.size())
            ? segment_entry_time[next_idx]
            : t_exit;
        ++segment_idx;
    }

    return pos;
}

Point RouteFollower::MoveBackward(const double dx) {
    if (dx <= 0.0) {
        return pos;
    }
    x -= dx;
    if (x < 0.0) {
        x = 0.0;
    }
    if (!cumulative_len.empty()) {
        while (segment_idx > 0 && x < cumulative_len[segment_idx]) {
            --segment_idx;
            // Restore segment_t_enter for the segment we backed into. Its
            // first-forward-entry time is preserved in segment_entry_time so
            // future forward re-crossings reuse it.
            if (segment_idx < segment_entry_time.size()) {
                segment_t_enter = segment_entry_time[segment_idx];
            }
        }
    }
    bg::line_interpolate(route, x, pos);
    RebuildIncomingRoute();
    return pos;
}

Point RouteFollower::MoveBackward(const double dx, const double dt,
                                  const double t_now, const double max_dx) {
    (void)dt;
    (void)t_now;
    const double dx_capped = std::min(dx, max_dx);
    if (dx_capped <= 0.0) {
        return pos;
    }
    return MoveBackward(dx_capped);
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
    if (state == move) {
        pos = route_follower.Move(speed * dt);
        if (route_follower.IsFinished()) {
            state = idle;
        }
    } else if (state == reverse) {
        pos = route_follower.MoveBackward(kReverseSpeedFactor * speed * dt);
    }
}

void Agent::Move(const double t, const double dt, const double speed) {
    if (state == move) {
        pos = route_follower.Move(speed * dt, dt, t);
        if (route_follower.IsFinished()) {
            state = idle;
        }
    } else if (state == reverse) {
        pos = route_follower.MoveBackward(kReverseSpeedFactor * speed * dt);
    }
}

void Agent::Move(const double t, const double dt, const double speed,
                 const double max_dx) {
    if (state == move) {
        const double dx = speed * dt;
        if (dx <= 0.0) {
            return;
        }
        if (route_follower.vertex_ids.empty() || max_dx >= dx) {
            pos = route_follower.Move(dx, dt, t);
        } else {
            pos = route_follower.Move(dx, dt, t, max_dx);
        }
        if (route_follower.IsFinished()) {
            state = idle;
        }
    } else if (state == reverse) {
        const double dx = kReverseSpeedFactor * speed * dt;
        if (dx <= 0.0) {
            return;
        }
        if (max_dx >= dx) {
            pos = route_follower.MoveBackward(dx);
        } else {
            pos = route_follower.MoveBackward(dx, dt, t, max_dx);
        }
    }
}
