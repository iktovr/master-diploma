#include "semaphore.h"

#include <memory>
#include <utility>

#include "agent.h"
#include "geometry.h"
#include "graph.h"
#include "statistics.h"

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

SemaphoreManager::SemaphoreManager(std::shared_ptr<const Graph> graph_) : graph(std::move(graph_)) {
    for (int u = 0; u < static_cast<int>(graph->vertices.size()); ++u) {
        for (auto& [v, edge] : graph->edges[u]) {
            if (v > u) {
                continue;
            }
            if (edge.narrow) {
                semaphores.emplace_back(graph->vertices[u].pos, graph->vertices[v].pos);
            }
        }
    }

    std::vector<IndexedSegment> packed;
    packed.reserve(semaphores.size());
    for (int i = 0; i < static_cast<int>(semaphores.size()); ++i) {
        Box bbox;
        bg::envelope(semaphores[i].segment, bbox);
        packed.emplace_back(bbox, i);
    }
    index = RTree(packed.begin(), packed.end());
}

void SemaphoreManager::Step(const double t, Agents& agents) {
    for (size_t i = 0; i < agents.size(); ++i) {
        auto& agent = agents[i];
        if (agent.state != Agent::move) {
            continue;
        }

        const int agent_id = static_cast<int>(i);
        const auto& route = agent.IncomingRoute();

        auto inside_it = agent_to_sem.find(agent_id);
        if (inside_it != agent_to_sem.end()) {
            auto& sem = semaphores[inside_it->second];
            if (sem.Intersection(route) >= 0) {
                sem.Exit(agent_id);
                agent_to_sem.erase(inside_it);
            }
            continue;
        }

        Linestring edge{route[1], route[2]};
        Box query_box;
        bg::envelope(edge, query_box);

        for (auto qit = index.qbegin(bgi::intersects(query_box));
             qit != index.qend(); ++qit) {
            auto& sem = semaphores[qit->second];
            int intersection = sem.Intersection(route);
            if (intersection == 1) {
                if (bg::length(Linestring{route[0], route[1]}) < 0.25) {
                    sem.left_q.emplace(t, agent_id);
                    agent.state = Agent::wait;
                }
            } else if (intersection == 2) {
                if (bg::length(Linestring{route[0], route[1]}) < 0.25) {
                    sem.right_q.emplace(t, agent_id);
                    agent.state = Agent::wait;
                }
            }
        }
    }

    for (size_t sem_idx = 0; sem_idx < semaphores.size(); ++sem_idx) {
        auto& sem = semaphores[sem_idx];
        if (sem.Empty()) {
            bool left = (sem.left_q.size() > sem.right_q.size() || (sem.left_q.size() == sem.right_q.size() && sem.left_q.size() > 0 && sem.left_q.top().first < sem.right_q.top().first));
            auto& queue = left ? sem.left_q : sem.right_q;
            auto& inner_queue = left ? sem.inner_left_q : sem.inner_right_q;
            while (!queue.empty()) {
                auto [add_t, a] = queue.top();
                if (add_t < t) {
                    Statistics::Get().waiting_time.Add(t - add_t);
                }
                queue.pop();
                agents[a].state = Agent::move;
                inner_queue.emplace(a);
                agent_to_sem[a] = static_cast<int>(sem_idx);
            }
        } else {
            if (!sem.inner_left_q.empty()) {
                while (!sem.left_q.empty()) {
                    auto [add_t, a] = sem.left_q.top();
                    if (add_t < t) {
                        Statistics::Get().waiting_time.Add(t - add_t);
                    }
                    sem.left_q.pop();
                    agents[a].state = Agent::move;
                    sem.inner_left_q.emplace(a);
                    agent_to_sem[a] = static_cast<int>(sem_idx);
                }
            }
            if (!sem.inner_right_q.empty()) {
                while (!sem.right_q.empty()) {
                    auto [add_t, a] = sem.right_q.top();
                    if (add_t < t) {
                        Statistics::Get().waiting_time.Add(t - add_t);
                    }
                    sem.right_q.pop();
                    agents[a].state = Agent::move;
                    sem.inner_right_q.emplace(a);
                    agent_to_sem[a] = static_cast<int>(sem_idx);
                }
            }
        }
    }
}
