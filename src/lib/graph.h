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
        bool narrow = false;

        bool operator==(const Edge& other) const = default;
    };

    Graph() = default;

    void AddVertex(const double x, const double y, const Vertex::Type type = Vertex::none);

    void AddEdge(const int u, const int v, const bool narrow = false);

    inline double Distance(const int u, const int v) const {
        return bg::distance(vertices[u].pos, vertices[v].pos);
    }

    std::vector<int> GetVertices(const std::optional<Vertex::Type> type = std::nullopt) const;

    double Width() const;

    double Height() const;

    Point Centroid() const;

    static Graph LoadFromFile(const std::filesystem::path path);

    static Graph LoadFromGeoJsonFile(const std::filesystem::path path, int basepoints_limit = -1);

    std::vector<int> Search(const int u, const int v) const;
    Linestring GetRoute(const int u, const int v) const;

protected:
    int vertex_id = 0;

public:
    std::vector<Vertex> vertices;
    std::vector<std::unordered_map<int, Edge>> edges;
};
