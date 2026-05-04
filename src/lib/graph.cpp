#include "graph.h"

#include <algorithm>
#include <cassert>
#include <functional>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

using SearchState = std::pair<double, int>;

struct VertexData {
    double dist;
    double cost;
    int prev;
};

std::vector<int> Graph::Search(const int start, const int finish) {
    std::unordered_map<int, VertexData> ctx;
    std::priority_queue<SearchState, std::vector<SearchState>, std::greater<SearchState>> heap;

    ctx[start] = {0, Distance(start, finish), start};
    heap.push({ctx[start].cost, start});

    int cur;
    while (!heap.empty()) {
        std::tie(std::ignore, cur) = heap.top();
        heap.pop();

        if (cur == finish) {
            break;
        }

        auto& cur_data = ctx[cur];

        for (const auto& [next, edge] : edges[cur]) {
            if (next == cur_data.prev) {
                continue;
            }
            double next_dist = cur_data.dist + edge.length;
            double next_cost = next_dist + Distance(next, finish);
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

Linestring Graph::GetRoute(const int u, const int v) {
    const auto path = Search(u, v);
    Linestring route;
    route.reserve(path.size());
    for (const auto i : path) {
        route.push_back(vertices[i].pos);
    }
    return route;
}
