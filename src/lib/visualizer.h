#pragma once

#include <cassert>
#include <filesystem>
#include <format>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include "agent.h"
#include "geometry.h"
#include "graph.h"

namespace fs = std::filesystem;

class Visualizer {
public:
    Visualizer(const double width_, const double height_, const Point& center_, const int max_dimension_in_pixels_, const std::string directory_)
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

    void SavePersistentPart() {
        persistent_img = img.clone();
    }

    void ClearFrame() {
        persistent_img.copyTo(img);
    }

    void DrawAgent(const Agent& agent) {
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

    void DrawGraph(const Graph& graph) {
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

    void SaveFrame() {
        cv::imwrite(directory / std::format("frame_{:04}.png", frame++), img);
    }

protected:
    double width;
    double height;
    Point center;
    int img_width;
    int img_height;
    fs::path directory;
    cv::Mat img;
    cv::Mat persistent_img;
    int frame = 0;

    inline int ToPixels(const double x) const {
        return static_cast<int>(x / width * img_width);
    }

    inline cv::Point ToPixels(const double x, const double y) const {
        return {static_cast<int>((x - center.x() + width / 2) / width * img_width), static_cast<int>((-y + center.y() + height / 2) / height * img_height)};
    }

    inline cv::Point ToPixels(const Point& point) const {
        return ToPixels(point.x(), point.y());
    }
};
