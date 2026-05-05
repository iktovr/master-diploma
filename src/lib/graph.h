#pragma once

#include <cmath>
#include <filesystem>
#include <optional>
#include <unordered_map>
#include <vector>

#include "geometry.h"

class Graph {
public:
    struct Vertex {
        enum Type {
            none,
            base,
            delivery
        };

        int id;
        Point pos;
        Type type = none;

        bool operator==(const Vertex& other) const = default;
    };

    struct Edge {
        double length;

        bool operator==(const Edge& other) const = default;
    };

    Graph() = default;

    void AddVertex(const double x, const double y, const Vertex::Type type = Vertex::none) {
        vertices.emplace_back(vertex_id++, Point{x, y}, type);
        edges.emplace_back();
    }

    void AddEdge(const int u, const int v) {
        double length = Distance(u, v);
        edges[u].emplace(v, Edge{length});
        edges[v].emplace(u, Edge{length});
    }

    inline double Distance(const int u, const int v) const {
        return bg::distance(vertices[u].pos, vertices[v].pos);
    }

    std::vector<int> GetVertices(const std::optional<Vertex::Type> type = std::nullopt) const {
        std::vector<int> res;
        for (const auto& v : vertices) {
            if (!type || v.type == *type) {
                res.push_back(v.id);
            }
        }
        return res;
    }

    double Width() const {
        auto [min, max] = std::minmax_element(
            vertices.begin(), vertices.end(),
            [](const auto &u, const auto &v) { return u.pos.x() < v.pos.x(); });
        return abs(min->pos.x()) + abs(max->pos.x());
    }

    double Height() const {
        auto [min, max] = std::minmax_element(
            vertices.begin(), vertices.end(),
            [](const auto &u, const auto &v) { return u.pos.y() < v.pos.y(); });
        return abs(min->pos.y()) + abs(max->pos.y());
    }

    Point Centroid() const {
        auto [min_x, max_x] = std::minmax_element(
            vertices.begin(), vertices.end(),
            [](const auto &u, const auto &v) { return u.pos.x() < v.pos.x(); });
        auto [min_y, max_y] = std::minmax_element(
            vertices.begin(), vertices.end(),
            [](const auto &u, const auto &v) { return u.pos.y() < v.pos.y(); });
        return {(min_x->pos.x() + max_x->pos.x()) / 2, (min_y->pos.y() + max_y->pos.y()) / 2};
    }

    static Graph LoadFromFile(const std::filesystem::path path);

    std::vector<int> Search(const int u, const int v) const;
    Linestring GetRoute(const int u, const int v) const;

protected:
    int vertex_id = 0;

public:
    std::vector<Vertex> vertices;
    std::vector<std::unordered_map<int, Edge>> edges;
};
