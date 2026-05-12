#include "graph.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <filesystem>
#include <functional>
#include <fstream>
#include <queue>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#include <proj.h>

namespace fs = std::filesystem;

void Graph::AddVertex(const double x, const double y, const Vertex::Type type) {
    vertices.emplace_back(vertex_id++, Point{x, y}, type);
    edges.emplace_back();
}

void Graph::AddEdge(const int u, const int v, const bool narrow) {
    double length = Distance(u, v);
    edges[u].emplace(v, Edge{length, narrow});
    edges[v].emplace(u, Edge{length, narrow});
}

std::vector<int> Graph::GetVertices(const std::optional<Vertex::Type> type) const {
    std::vector<int> res;
    for (const auto& v : vertices) {
        if (!type || v.type == *type) {
            res.push_back(v.id);
        }
    }
    return res;
}

double Graph::Width() const {
    auto [min, max] = std::minmax_element(
        vertices.begin(), vertices.end(),
        [](const auto &u, const auto &v) { return u.pos.x() < v.pos.x(); });
    return std::abs(min->pos.x()) + std::abs(max->pos.x());
}

double Graph::Height() const {
    auto [min, max] = std::minmax_element(
        vertices.begin(), vertices.end(),
        [](const auto &u, const auto &v) { return u.pos.y() < v.pos.y(); });
    return std::abs(min->pos.y()) + std::abs(max->pos.y());
}

Point Graph::Centroid() const {
    auto [min_x, max_x] = std::minmax_element(
        vertices.begin(), vertices.end(),
        [](const auto &u, const auto &v) { return u.pos.x() < v.pos.x(); });
    auto [min_y, max_y] = std::minmax_element(
        vertices.begin(), vertices.end(),
        [](const auto &u, const auto &v) { return u.pos.y() < v.pos.y(); });
    return {(min_x->pos.x() + max_x->pos.x()) / 2, (min_y->pos.y() + max_y->pos.y()) / 2};
}

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

namespace {

// Coordinate key rounded to 1e-7 degrees (~1 cm precision at equator).
struct LonLat {
    long long lon7;
    long long lat7;

    bool operator==(const LonLat& o) const {
        return lon7 == o.lon7 && lat7 == o.lat7;
    }

    static LonLat from(double lon, double lat) {
        return {std::llround(lon * 1e7), std::llround(lat * 1e7)};
    }

    double lon() const { return lon7 * 1e-7; }
    double lat() const { return lat7 * 1e-7; }
};

struct LonLatHash {
    std::size_t operator()(const LonLat& p) const {
        auto h1 = std::hash<long long>{}(p.lon7);
        auto h2 = std::hash<long long>{}(p.lat7);
        return h1 ^ (h2 * 2654435761ULL);
    }
};

} // namespace

Graph Graph::LoadFromGeoJsonFile(const fs::path path) {
    assert(fs::exists(path) && fs::is_regular_file(path));

    std::ifstream file(path);
    assert(file.is_open());

    using json = nlohmann::json;
    const json root = json::parse(file);

    struct PointInfo {
        LonLat pos;
        Vertex::Type type;
    };
    std::unordered_map<LonLat, Vertex::Type, LonLatHash> point_type_by_pos;

    for (const auto& feature : root.at("features")) {
        const auto& geom = feature.at("geometry");
        if (geom.at("type").get<std::string>() != "Point") {
            continue;
        }
        const auto& coords = geom.at("coordinates");
        LonLat pos = LonLat::from(coords[0].get<double>(), coords[1].get<double>());

        Vertex::Type vtype = Vertex::none;
        const auto& props = feature.at("properties");
        if (props.contains("type")) {
            const auto t = props.at("type").get<std::string>();
            if (t == "base_point") {
                vtype = Vertex::base;
            } else if (t == "delivery_point") {
                vtype = Vertex::delivery;
            }
        }
        point_type_by_pos[pos] = vtype;
    }

    struct LineEdge {
        LonLat a;
        LonLat b;
        bool narrow = false;
    };
    std::vector<LineEdge> lines;

    for (const auto& feature : root.at("features")) {
        const auto& geom = feature.at("geometry");
        if (geom.at("type").get<std::string>() != "LineString") {
            continue;
        }
        const auto& coords = geom.at("coordinates");
        assert(coords.size() == 2);
        LineEdge le{
            LonLat::from(coords[0][0].get<double>(), coords[0][1].get<double>()),
            LonLat::from(coords[1][0].get<double>(), coords[1][1].get<double>()),
            false,
        };
        const auto& props = feature.at("properties");
        if (props.contains("narrow")) {
            const auto v = props.at("narrow").get<std::string>();
            assert(v == "yes" || v == "no");
            le.narrow = (v == "yes");
        }
        lines.push_back(le);
    }

    std::unordered_map<LonLat, int, LonLatHash> coord_to_id;

    auto ensure_vertex = [&](const LonLat& ll) {
        if (!coord_to_id.count(ll)) {
            coord_to_id[ll] = static_cast<int>(coord_to_id.size());
        }
    };

    for (const auto& le : lines) {
        ensure_vertex(le.a);
        ensure_vertex(le.b);
    }
    for (const auto& [pos, type] : point_type_by_pos) {
        ensure_vertex(pos);
    }

    const int n = static_cast<int>(coord_to_id.size());
    std::vector<LonLat> id_to_coord(n);
    for (const auto& [ll, id] : coord_to_id) {
        id_to_coord[id] = ll;
    }

    double sum_lon = 0.0;
    for (const auto& ll : id_to_coord) {
        sum_lon += ll.lon();
    }
    const double mean_lon = sum_lon / n;
    const int utm_zone = static_cast<int>((mean_lon + 180.0) / 6.0) + 1;

    const std::string proj_str =
        "+proj=utm +zone=" + std::to_string(utm_zone) + " +datum=WGS84 +units=m +no_defs";

    PJ_CONTEXT* ctx = proj_context_create();
    PJ* P = proj_create(ctx, proj_str.c_str());
    assert(P != nullptr);

    std::vector<std::pair<double, double>> utm_coords(n);
    double sum_x = 0.0, sum_y = 0.0;
    for (int i = 0; i < n; ++i) {
        const LonLat& ll = id_to_coord[i];
        PJ_COORD c_in = proj_coord(
            proj_torad(ll.lon()),
            proj_torad(ll.lat()),
            0.0, 0.0);
        PJ_COORD c_out = proj_trans(P, PJ_FWD, c_in);
        utm_coords[i] = {c_out.xy.x, c_out.xy.y};
        sum_x += c_out.xy.x;
        sum_y += c_out.xy.y;
    }

    proj_destroy(P);
    proj_context_destroy(ctx);

    const double ref_x = sum_x / n;
    const double ref_y = sum_y / n;

    Graph g;
    for (int i = 0; i < n; ++i) {
        Vertex::Type vtype = Vertex::none;
        auto pit = point_type_by_pos.find(id_to_coord[i]);
        if (pit != point_type_by_pos.end()) {
            vtype = pit->second;
        }
        g.AddVertex(utm_coords[i].first - ref_x, utm_coords[i].second - ref_y, vtype);
    }

    for (const auto& le : lines) {
        int u = coord_to_id.at(le.a);
        int v = coord_to_id.at(le.b);
        if (u != v && !g.edges[u].count(v)) {
            g.AddEdge(u, v, le.narrow);
        }
    }

    return g;
}