#include "visualizer.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <filesystem>

#include "statistics.h"

namespace {

constexpr double kPixelsPerWorldUnit = 2.0;
constexpr int    kMinFrame = 1000;
constexpr int    kMaxFrame = 10000;

constexpr double kAgentRadiusRatio         = 0.006;
constexpr double kVertexRadiusRatio        = 0.003;
constexpr double kEdgeThicknessRatio       = 0.0010;
constexpr double kNarrowEdgeThicknessRatio = 0.0025;
constexpr double kMarginRatio              = 0.020;
constexpr double kAgentLaneOffsetRatio     = 0.5;

constexpr int    kMinPrimitivePx = 1;

inline int RatioPx(int ref_px, double ratio) {
    return std::max(kMinPrimitivePx, static_cast<int>(std::round(ref_px * ratio)));
}

inline cv::Scalar SpeedColor(double speed, double max_speed) {
    if (!(max_speed > 0.0)) {
        return cv::Scalar(0, 0, 0);
    }
    double ratio = speed / max_speed;
    if (ratio < 0.0) ratio = 0.0;
    if (ratio > 1.0) ratio = 1.0;
    double b = 0.0, g = 0.0, r = 0.0;
    if (ratio < 0.5) {
        // red -> yellow
        const double t = ratio / 0.5;
        r = 255.0;
        g = 255.0 * t;
    } else {
        // yellow -> green
        const double t = (ratio - 0.5) / 0.5;
        r = 255.0 * (1.0 - t);
        g = 255.0;
    }
    return cv::Scalar(b, g, r);
}

inline cv::Scalar AgentStateColor(Agent::State state) {
    switch (state) {
        case Agent::move:
            return cv::Scalar(255, 255, 255);  // white
        case Agent::idle:
            return cv::Scalar(186, 186, 186);  // gray
        case Agent::wait:
            return cv::Scalar(0, 186, 186);   // dark yellow
        case Agent::reverse:
            return cv::Scalar(0, 0, 186);     // dark red
        default:
            return cv::Scalar(255, 255, 255);  // white as default
    }
}

}  // namespace

int ComputeAutoFrameSize(const Graph& graph) {
    if (graph.vertices.empty()) {
        return kMinFrame;
    }
    const double max_extent = std::max(graph.Width(), graph.Height());
    if (max_extent <= 0.0) {
        return kMinFrame;
    }
    const double target = kPixelsPerWorldUnit * max_extent;
    return std::clamp(static_cast<int>(std::ceil(target)), kMinFrame, kMaxFrame);
}

Visualizer::Visualizer(const double width_, const double height_, const Point& center_,
                       const int max_dimension_in_pixels_, const double objects_scale_, const std::string directory_,
                       const double max_speed_, bool draw_graph_, bool draw_agents_, bool draw_statistics_, bool draw_points_, bool draw_narrow_edges_)
    : center(center_), max_speed(max_speed_), directory(directory_),
      draw_graph(draw_graph_), draw_agents(draw_agents_), draw_statistics(draw_statistics_), draw_points(draw_points_), draw_narrow_edges(draw_narrow_edges_) {
    assert(fs::exists(directory));
    assert(fs::is_directory(directory));

    if (width_ < height_) {
        img_height = max_dimension_in_pixels_;
        img_width  = static_cast<int>(width_ / height_ * img_height);
    } else {
        img_width  = max_dimension_in_pixels_;
        img_height = static_cast<int>(height_ / width_ * img_width);
    }

    const int ref_px = std::max(img_width, img_height);
    agent_radius_px          = RatioPx(ref_px, kAgentRadiusRatio * objects_scale_);
    vertex_radius_px         = RatioPx(ref_px, kVertexRadiusRatio * objects_scale_);
    edge_thickness_px        = RatioPx(ref_px, kEdgeThicknessRatio * objects_scale_);
    narrow_edge_thickness_px = RatioPx(ref_px, kNarrowEdgeThicknessRatio * objects_scale_);

    const int margin_px = std::max(agent_radius_px + 2,
                                   static_cast<int>(std::round(ref_px * kMarginRatio)));
    const double pad_w = static_cast<double>(margin_px)
                       / std::max(1, img_width  - 2 * margin_px) * width_;
    const double pad_h = static_cast<double>(margin_px)
                       / std::max(1, img_height - 2 * margin_px) * height_;
    width  = width_  + 2 * pad_w;
    height = height_ + 2 * pad_h;

    img = cv::Mat(img_height, img_width, CV_8UC3, cv::Scalar(255, 255, 255));
}

void Visualizer::DrawAgent(const Agent& agent) {
    if (!draw_agents) {
        return;
    }
    
    const static cv::Scalar edge_color(0, 0, 0);
    const cv::Scalar fill_color = AgentStateColor(agent.state);

    cv::Point center_px = ToPixels(agent.pos);

    // apply lane offset for moving or waiting agents
    double dir_x = 0.0, dir_y = 0.0;
    bool has_direction = false;
    
    if (agent.state != Agent::idle && agent.route_follower.route.size() >= 2) {
        const auto& route = agent.route_follower.route;
        size_t segment_idx = agent.route_follower.segment_idx;
        
        if (segment_idx >= route.size() - 1) {
            segment_idx = route.size() - 2;
        }
        
        if (segment_idx < route.size() - 1) {
            cv::Point a = ToPixels(route[segment_idx]);
            cv::Point b = ToPixels(route[segment_idx + 1]);
            
            double dx = b.x - a.x;
            double dy = b.y - a.y;
            double length = std::sqrt(dx * dx + dy * dy);
            
            if (length > 0.0) {
                double nx = -dy / length;
                double ny = dx / length;
                
                int offset_px = static_cast<int>(agent_radius_px * kAgentLaneOffsetRatio);
                center_px.x += static_cast<int>(nx * offset_px);
                center_px.y += static_cast<int>(ny * offset_px);
                
                // Store direction for the directional line
                dir_x = dx / length;
                dir_y = dy / length;
                has_direction = true;
            }
        }
    }

    cv::circle(img, center_px, agent_radius_px, fill_color, cv::FILLED);
    cv::circle(img, center_px, agent_radius_px, edge_color);

    // Draw directional line from center to edge, codirectional with route
    if (has_direction) {
        cv::Point end_px;
        end_px.x = center_px.x + static_cast<int>(dir_x * agent_radius_px);
        end_px.y = center_px.y + static_cast<int>(dir_y * agent_radius_px);
        cv::line(img, center_px, end_px, edge_color, 1);
    }

    // if (agent.state != Agent::idle) {
    //     const auto& route = agent.IncomingRoute();
    //     for (size_t i = 0; i < route.size() - 1; ++i) {
    //         cv::line(img, ToPixels(route[i]), ToPixels(route[i+1]), cv::Scalar{0, 0, 255}, 3);
    //     }
    // }
}

void Visualizer::DrawGraph(const Graph& graph) {
    if (!draw_graph) {
        return;
    }
    
    const static cv::Scalar edge_color(0, 0, 0);
    const static cv::Scalar narrow_edge_color(255, 0, 0);
    const static cv::Scalar vertex_color(0, 0, 0);
    const static cv::Scalar base_color(0, 0, 255);
    const static cv::Scalar delivery_color(0, 255, 0);

    for (int u = 0; u < static_cast<int>(graph.vertices.size()); ++u) {
        for (auto& [v, edge] : graph.edges[u]) {
            if (v > u) {
                continue;
            }
            if (edge.narrow && draw_narrow_edges) {
                cv::line(img, ToPixels(graph.vertices[u].pos), ToPixels(graph.vertices[v].pos), narrow_edge_color, narrow_edge_thickness_px);
            }
            cv::line(img, ToPixels(graph.vertices[u].pos), ToPixels(graph.vertices[v].pos), edge_color, edge_thickness_px);
        }
    }
    
    if (draw_points) {
        for (const auto& v : graph.vertices) {
            if (v.type == Graph::Vertex::none) {
                continue;
            }
            cv::Scalar color = vertex_color;
            if (v.type == Graph::Vertex::base) {
                color = base_color;
            } else if (v.type == Graph::Vertex::delivery) {
                color = delivery_color;
            }
            cv::circle(img, ToPixels(v.pos), vertex_radius_px, color, cv::FILLED);
        }
    }
}

void Visualizer::DrawGraphStatistics(const Graph& graph, GraphEdgeStatistics& stats, double t_now) {
    if (!draw_statistics) {
        return;
    }
    if (!(max_speed > 0.0)) {
        return;
    }
    for (int u = 0; u < static_cast<int>(graph.vertices.size()); ++u) {
        for (auto& [v, edge] : graph.edges[u]) {
            if (v > u) {
                continue;
            }
            const double avg_speed = stats.AverageSpeed(u, v, t_now);
            if (!stats.Has(u, v)) {
                continue;
            }
            cv::line(img, ToPixels(graph.vertices[u].pos),
                     ToPixels(graph.vertices[v].pos),
                     SpeedColor(avg_speed, max_speed), edge_thickness_px);
        }
    }
}
