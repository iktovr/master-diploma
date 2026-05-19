#pragma once

#include <memory>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "graph.h"
#include "agent.h"
#include "geometry.h"

struct SemaphoreManager {
    struct Semaphore {
        using QueuedAgent = std::pair<double, int>;
        using Queue = std::priority_queue<QueuedAgent, std::vector<QueuedAgent>, std::greater<QueuedAgent>>;

        Linestring segment;
        double length;
        Queue left_q;
        Queue right_q;
        std::unordered_set<int> inner_left_q;
        std::unordered_set<int> inner_right_q;

        Semaphore(const Point& u, const Point& v) : segment{u, v}, length(bg::length(segment)) {}

        int Intersection(const Linestring& incoming_route) const;

        bool Inside(const int& id) const {
            return inner_left_q.contains(id) || inner_right_q.contains(id);
        }

        void Exit(const int& id) {
            inner_left_q.erase(id);
            inner_right_q.erase(id);
        }

        bool Empty() const {
            return inner_left_q.empty() && inner_right_q.empty();
        }
    };

    using IndexedSegment = std::pair<Box, int>;
    using RTree = bgi::rtree<IndexedSegment, bgi::rstar<16>>;

    std::shared_ptr<const Graph> graph;
    std::vector<Semaphore> semaphores;
    RTree index;
    std::unordered_map<int, int> agent_to_sem;

    SemaphoreManager(std::shared_ptr<const Graph> graph);

    void Step(const double t, Agents& agents);
};