#pragma once

#include <cmath>
#include <unordered_map>
#include <vector>

class Graph {
public:
    struct Vertex {
        int id;
        double x;
        double y;
    };

    struct Edge {
        double length;
    };

    Graph() = default;

    void AddVertex(double x, double y) {
        vertices.emplace_back(vertex_id++, x, y);
        edges.emplace_back();
    }

    void AddEdge(int u, int v) {
        double length = Distance(u, v);
        edges[u].emplace(v, Edge{length});
        edges[v].emplace(u, Edge{length});
    }

    inline double Distance(int u, int v) {
        return std::sqrt(std::pow(vertices[u].x - vertices[v].x, 2) + std::pow(vertices[u].y - vertices[v].y, 2));
    }

    std::vector<int> Search(int u, int v);

protected:
    int vertex_id = 0;

public:
    std::vector<Vertex> vertices;
    std::vector<std::unordered_map<int, Edge>> edges;
};