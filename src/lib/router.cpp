#include "router.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <functional>
#include <queue>
#include <unordered_map>
#include <vector>

namespace lib {

std::vector<int> AStarRouter::Search(const int start, const int finish, const double t) const {
    std::unordered_map<int, VertexData> ctx;
    std::priority_queue<SearchState, std::vector<SearchState>, std::greater<SearchState>> heap;

    ctx[start] = {0, Heuristic(start, finish), start};
    heap.push({ctx[start].cost, start});

    int cur;
    while (!heap.empty()) {
        std::tie(std::ignore, cur) = heap.top();
        heap.pop();

        if (cur == finish) {
            break;
        }

        auto& cur_data = ctx[cur];

        for (const auto& [next, edge] : graph_->edges[cur]) {
            if (next == cur_data.prev) {
                continue;
            }
            double next_dist = cur_data.dist + Cost(cur, next, edge, t);
            double next_cost = next_dist + Heuristic(next, finish);
            if (!ctx.contains(next) || next_cost < ctx[next].cost) {
                heap.emplace(next_cost, next);
                ctx[next] = {next_dist, next_cost, cur};
            }
        }
    }

    assert(cur == finish);

    std::vector<int> res = {cur};
    while (cur != start) {
        cur = ctx[cur].prev;
        res.push_back(cur);
    }

    std::reverse(res.begin(), res.end());
    return res;
}

Linestring AStarRouter::GetRoute(const int u, const int v, const double t,
                                 const int /*agent_id*/) const {
    const auto path = Search(u, v, t);
    Linestring route;
    route.reserve(path.size());
    for (const auto i : path) {
        route.push_back(graph_->vertices[i].pos);
    }
    return route;
}

std::pair<Linestring, std::vector<int>>
AStarRouter::GetRouteWithVertices(const int u, const int v, const double t,
                                  const int /*agent_id*/) const {
    auto path = Search(u, v, t);
    Linestring route;
    route.reserve(path.size());
    for (const auto i : path) {
        route.push_back(graph_->vertices[i].pos);
    }
    return {std::move(route), std::move(path)};
}

}
