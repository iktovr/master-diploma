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
    Visualizer(const int img_width, const int img_height, const double width, const double height, const std::string directory)
        : img_width(img_width), img_height(img_height), width(width), height(height), directory(directory), img(img_width, img_height, CV_8UC3, cv::Scalar(255, 255, 255)) {
        assert(fs::exists(directory));
        assert(fs::is_directory(directory));
    }

    void ClearFrame() {
        // TODO: keep persistent part of image
        img = cv::Scalar(255, 255, 255);
    }

    void DrawAgent(const Agent& agent) {
        const static cv::Scalar edge_color(0, 0, 0);
        const static cv::Scalar fill_color(255, 255, 255);

        cv::circle(img, ToPixels(agent.pos), ToPixels(0.1), fill_color, cv::FILLED);
        cv::circle(img, ToPixels(agent.pos), ToPixels(0.1), edge_color);
    }

    void DrawGraph(const Graph& graph) {
        const static cv::Scalar color(0, 0, 0);
        const static cv::Scalar base_color(0, 0, 255);
        const static cv::Scalar delivery_color(0, 255, 0);

        for (int u = 0; u < static_cast<int>(graph.vertices.size()); ++u) {
            for (auto& [v, edge] : graph.edges[u]) {
                if (v < u) {
                    cv::line(img, ToPixels(graph.vertices[u].pos), ToPixels(graph.vertices[v].pos), color);
                }
            }
        }
        for (const auto& v : graph.vertices) {
            cv::Scalar vertex_color;
            if (v.type == Graph::Vertex::base) {
                vertex_color = base_color;
            } else if (v.type == Graph::Vertex::delivery) {
                vertex_color = delivery_color;
            }
            cv::circle(img, ToPixels(v.pos), 4, vertex_color, cv::FILLED);
        }
    }

    void SaveFrame() {
        cv::imwrite(directory / std::format("frame_{:04}.png", frame++), img);
    }

protected:
    int img_width;
    int img_height;
    double width;
    double height;
    fs::path directory;
    cv::Mat img;
    int frame = 0;

    inline int ToPixels(const double x) const {
        return static_cast<int>(x / height * img_height);
    }

    inline cv::Point ToPixels(const double x, const double y) const {
        return {static_cast<int>((x + height / 2) / height * img_height), static_cast<int>((-y + width / 2) / width * img_width)};
    }

    inline cv::Point ToPixels(const Point& point) const {
        return ToPixels(point.x(), point.y());
    }
};