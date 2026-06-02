#ifndef MAP_H
#define MAP_H

#include <iostream>
#include <string>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <vector>
#include <unordered_set>
#include "tinyxml2.h"
#include "const.h"
#include "structs.h"

class Map
{
private:
    std::vector<std::vector<int>> grid;
    std::vector<gNode> nodes;
    std::vector<std::vector<Node>> valid_moves;
    int  height, width, size;
    int  connectedness;
    double agent_size;
    bool map_is_roadmap;
    bool check_line(int x1, int y1, int x2, int y2);
    bool get_grid(const char* FileName);
    bool get_roadmap(const char* FileName);

    // Undirected narrow-edge set. Stored as packed (min,max) keys so the
    // direction does not matter. When empty, narrow-edge gating is
    // disabled and CCBS treats every edge as conflict-eligible (legacy
    // behavior, used by the original XML test inputs).
    std::unordered_set<long long> narrow_edges;
    // Set of vertex ids that participate in at least one narrow edge.
    // Used to gate wait/goal-wait conflicts.
    std::unordered_set<int> narrow_vertices;

    static inline long long pack_edge_key(int a, int b)
    {
        long long lo = a < b ? a : b;
        long long hi = a < b ? b : a;
        return (lo << 32) | (hi & 0xffffffffLL);
    }
public:
    Map(double size, int k){ agent_size = size; connectedness = k; map_is_roadmap = false; height = 0; width = 0; this->size = 0; narrow_speed_factor_ = 1.0; }
    ~Map(){}
    int  get_size() const { return size; }
    bool get_map(const char* FileName);
    bool is_roadmap() const {return map_is_roadmap;}
    bool cell_is_obstacle(int i, int j) const;
    int  get_width() const {return width;}
    gNode get_gNode(int id) const {if(id < int(nodes.size())) return nodes[id]; return gNode();}
    int  get_id(int i, int j) const;
    double get_i (int id) const;
    double get_j (int id) const;
    std::vector<Node> get_valid_moves(int id) const;
    void print_map();
    void printPPM();

    // Programmatic roadmap construction (used by the in-tree CcbsRouter
    // adapter). Replaces the contents of nodes/valid_moves and switches
    // the map into roadmap mode without going through XML.
    // |adj| must have the same size as |nodes_in| and contain the list
    // of out-neighbors for every node id.
    void build_roadmap(const std::vector<gNode>& nodes_in,
                       const std::vector<std::vector<int>>& adj);

    // Narrow-edge management. When the narrow set is non-empty,
    // CCBS::check_conflict skips conflicts entirely outside narrow
    // edges (see cbs.cpp). An empty set is treated as "every edge is
    // narrow" → all conflicts considered, preserving legacy behavior.
    void set_narrow_edge(int u, int v)
    {
        narrow_edges.insert(pack_edge_key(u, v));
        narrow_vertices.insert(u);
        narrow_vertices.insert(v);
    }
    inline bool is_narrow_edge(int u, int v) const
    {
        return narrow_edges.find(pack_edge_key(u, v)) != narrow_edges.end();
    }
    inline bool has_narrow_at_vertex(int id) const
    {
        return narrow_vertices.find(id) != narrow_vertices.end();
    }
    inline bool has_narrow_set() const { return !narrow_edges.empty(); }

    // Velocity multiplier (relative to unit speed) applied while
    // traversing a narrow edge. Default 1.0 = no slowdown.
    // When set < 1.0, SIPP inflates the per-edge time-cost of every
    // narrow edge by 1/factor, so that the resulting CCBS paths
    // reach narrow edges at the correct lower speed and CCBS's
    // collision checks use the corresponding velocities.
    inline void set_narrow_speed_factor(double f) { narrow_speed_factor_ = (f > 0.0 ? f : 1.0); }
    inline double narrow_speed_factor() const { return narrow_speed_factor_; }

    // Returns the time-cost of traversing the directed edge u->v.
    // For wait moves (u==v) returns 0. For narrow edges the cost is
    // |distance| / narrow_speed_factor_. For all other edges it is
    // the plain Euclidean distance between nodes (unit speed).
    double edge_time_cost(int u, int v) const;

private:
    double narrow_speed_factor_;
};

#endif // MAP_H
