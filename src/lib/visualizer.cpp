#include "visualizer.h"

#include <cassert>
#include <filesystem>

Visualizer::Visualizer(const double width_, const double height_, const Point& center_, const int max_dimension_in_pixels_, const std::string directory_)
    : width(width_), height(height_), center(center_), directory(directory_) {
    assert(fs::exists(directory));
    assert(fs::is_directory(directory));
    if (width < height) {
        img_height = max_dimension_in_pixels_;
        img_width = width / height * img_height;
    } else {
        img_width = max_dimension_in_pixels_;
        img_height = height / width * img_width;
    }
    img = cv::Mat(img_height, img_width, CV_8UC3, cv::Scalar(255, 255, 255));
}

void Visualizer::DrawAgent(const Agent& agent) {
    const static cv::Scalar edge_color(0, 0, 0);
    const static cv::Scalar fill_color(255, 255, 255);

    cv::circle(img, ToPixels(agent.pos), 8, fill_color, cv::FILLED);
    cv::circle(img, ToPixels(agent.pos), 8, edge_color);

    // if (agent.state != Agent::idle) {
    //     const auto& route = agent.IncomingRoute();
    //     for (size_t i = 0; i < route.size() - 1; ++i) {
    //         cv::line(img, ToPixels(route[i]), ToPixels(route[i+1]), cv::Scalar{0, 0, 255}, 3);
    //     }
    // }
}

void Visualizer::DrawGraph(const Graph& graph) {
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
            if (edge.narrow) {
                cv::line(img, ToPixels(graph.vertices[u].pos), ToPixels(graph.vertices[v].pos), narrow_edge_color, 3);
            }
            cv::line(img, ToPixels(graph.vertices[u].pos), ToPixels(graph.vertices[v].pos), edge_color);
        }
    }
    for (const auto& v : graph.vertices) {
        cv::Scalar color = vertex_color;
        if (v.type == Graph::Vertex::base) {
            color = base_color;
        } else if (v.type == Graph::Vertex::delivery) {
            color = delivery_color;
        }
        cv::circle(img, ToPixels(v.pos), 4, color, cv::FILLED);
    }
}
