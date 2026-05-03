#pragma once

#include <cmath>
#include <unordered_map>
#include <vector>

#include "geometry.h"

class Graph {
public:
    struct Vertex {
        int id;
        Point pos;
    };

    struct Edge {
        double length;
    };

    Graph() = default;

    void AddVertex(double x, double y) {
        vertices.emplace_back(vertex_id++, Point{x, y});
        edges.emplace_back();
    }

    void AddEdge(int u, int v) {
        double length = Distance(u, v);
        edges[u].emplace(v, Edge{length});
        edges[v].emplace(u, Edge{length});
    }

    inline double Distance(int u, int v) {
        return bg::distance(vertices[u].pos, vertices[v].pos);
    }

    std::vector<int> Search(int u, int v);

protected:
    int vertex_id = 0;

public:
    std::vector<Vertex> vertices;
    std::vector<std::unordered_map<int, Edge>> edges;
};