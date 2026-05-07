#include "graph.h"

#include <algorithm>
#include <cassert>
#include <filesystem>
#include <functional>
#include <fstream>
#include <queue>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace fs = std::filesystem;

using SearchState = std::pair<double, int>;

struct VertexData {
    double dist;
    double cost;
    int prev;
};

std::vector<int> Graph::Search(const int start, const int finish) const {
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

Linestring Graph::GetRoute(const int u, const int v) const {
    const auto path = Search(u, v);
    Linestring route;
    route.reserve(path.size());
    for (const auto i : path) {
        route.push_back(vertices[i].pos);
    }
    return route;
}

Graph Graph::LoadFromFile(const fs::path path) {
    assert(fs::exists(path) && fs::is_regular_file(path));

    std::ifstream file(path);
    assert(file.is_open());

    Graph g;
    std::string s;
    while (std::getline(file, s) && !s.empty()) {
        double x, y;
        std::string type;
        std::istringstream ss(s);
        ss >> x >> y >> type;
        auto t = Graph::Vertex::none;
        if (type == "b") {
            t = Graph::Vertex::base;
        } else if (type == "d") {
            t = Graph::Vertex::delivery;
        }

        g.AddVertex(x, y, t);
    }

    assert(file);
    while (std::getline(file, s) && !s.empty()) {
        int u, v;
        std::string attr;
        bool narrow = false;
        std::istringstream ss(s);
        ss >> u >> v >> attr;
        if (attr == "n") {
            narrow = true;
        }

        g.AddEdge(u, v, narrow);
    }
    return g;
}