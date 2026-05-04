#pragma once

#include <cmath>
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
    };

    struct Edge {
        double length;
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

    std::vector<int> Search(const int u, const int v) const;
    Linestring GetRoute(const int u, const int v) const;

protected:
    int vertex_id = 0;

public:
    std::vector<Vertex> vertices;
    std::vector<std::unordered_map<int, Edge>> edges;
};