#pragma once

#include "graph.h"
#include "agent.h"
#include "geometry.h"

struct SemaphoreManager {
    struct Semaphore {
        
    };

    const Graph& graph;

    SemaphoreManager(const Graph& graph) : graph(graph) {}

    void Step(Agents& agents) {
        for (size_t i = 0; i < agents.size(); ++i) {
            auto& agent = agents[i];
            if (agent.state == Agent::idle) {
                continue;
            }

            
        }
    }
};