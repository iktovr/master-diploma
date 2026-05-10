#include "semaphore.h"

#include <utility>

int SemaphoreManager::Semaphore::Intersection(const Linestring& incoming_route) const {
    if (bg::equals(segment[0], incoming_route[1])) {
        if (bg::equals(segment[1], incoming_route[2])) {
            return 1; // incoming from left side
        }
        return -1; // agent inside, moving towards left side
    } else if (bg::equals(segment[1], incoming_route[1])) {
        if (bg::equals(segment[0], incoming_route[2])) {
            return 2; // incoming from right side
        }
        return -2; // agent inside, moving towards right side
    }
    return 0; // no intersection
}

SemaphoreManager::SemaphoreManager(const Graph& graph) : graph(graph) {
    for (int u = 0; u < static_cast<int>(graph.vertices.size()); ++u) {
        for (auto& [v, edge] : graph.edges[u]) {
            if (v > u) {
                continue;
            }
            if (edge.narrow) {
                semaphores.emplace_back(graph.vertices[u].pos, graph.vertices[v].pos);
            }
        }
    }
}

void SemaphoreManager::Step(const double t, Agents& agents) {
    for (size_t i = 0; i < agents.size(); ++i) {
        auto& agent = agents[i];
        if (agent.state != Agent::move) {
            continue;
        }

        for (auto& sem : semaphores) {
            int intersection = sem.Intersection(agent.IncomingRoute());
            if (sem.Inside(i)) {
                if (intersection >= 0) {
                    sem.Exit(i);
                }
                continue;
            }
            if (intersection == 0) {
                continue;
            } else if (intersection == 1) {
                if (bg::length(Linestring{agent.IncomingRoute()[0], agent.IncomingRoute()[1]}) < 0.25) {
                    sem.left_q.emplace(t, i);
                    agent.state = Agent::wait;
                }
            } else if (intersection == 2) {
                if (bg::length(Linestring{agent.IncomingRoute()[0], agent.IncomingRoute()[1]}) < 0.25) {
                    sem.right_q.emplace(t, i);
                    agent.state = Agent::wait;
                }
            } else {
                // double len = bg::length(Linestring{agent.IncomingRoute()[0], agent.IncomingRoute()[1]});
                // if (len < 0.5 * sem.length) {

                // }
            }
        }
    }

    for (auto& sem : semaphores) {
        if (sem.Empty()) {
            bool left = (sem.left_q.size() > sem.right_q.size() || (sem.left_q.size() == sem.right_q.size() && sem.left_q.size() > 0 && sem.left_q.top().first < sem.right_q.top().first));
            auto& queue = left ? sem.left_q : sem.right_q;
            auto& inner_queue = left ? sem.inner_left_q : sem.inner_right_q;
            while (!queue.empty()) {
                int a;
                std::tie(std::ignore, a) = queue.top();
                queue.pop();
                agents[a].state = Agent::move;
                inner_queue.emplace(a);
            }
        } else {
            if (!sem.inner_left_q.empty()) {
                while (!sem.left_q.empty()) {
                    int a;
                    std::tie(std::ignore, a) = sem.left_q.top();
                    sem.left_q.pop();
                    agents[a].state = Agent::move;
                    sem.inner_left_q.emplace(a);
                }
            }
            if (!sem.inner_right_q.empty()) {
                while (!sem.right_q.empty()) {
                    int a;
                    std::tie(std::ignore, a) = sem.right_q.top();
                    sem.right_q.pop();
                    agents[a].state = Agent::move;
                    sem.inner_right_q.emplace(a);
                }
            }
        }
    }
}
