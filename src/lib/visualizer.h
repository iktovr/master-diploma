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

class GraphEdgeStatistics;

int ComputeAutoFrameSize(const Graph& graph);

class Visualizer {
public:
    Visualizer(const double width_, const double height_, const Point& center_, const int max_dimension_in_pixels_, const double scale_, const std::string directory_, const double max_speed_ = 0.0,
               bool draw_graph_ = true, bool draw_agents_ = true, bool draw_statistics_ = false, bool draw_points_ = true, bool draw_narrow_edges_ = true);

    void SavePersistentPart() {
        persistent_img = img.clone();
    }

    void ClearFrame() {
        persistent_img.copyTo(img);
    }

    void DrawAgent(const Agent& agent);

    void DrawGraph(const Graph& graph);

    void DrawGraphStatistics(const Graph& graph, GraphEdgeStatistics& stats, double t_now);

    void SaveFrame() {
        cv::imwrite(directory / std::format("frame_{:04}.png", frame++), img);
    }

protected:
    double width;
    double height;
    Point center;
    int img_width;
    int img_height;
    int agent_radius_px;
    int vertex_radius_px;
    int edge_thickness_px;
    int narrow_edge_thickness_px;
    double max_speed;
    fs::path directory;
    cv::Mat img;
    cv::Mat persistent_img;
    int frame = 0;
    bool draw_graph = true;
    bool draw_agents = true;
    bool draw_statistics = false;
    bool draw_points = true;
    bool draw_narrow_edges = true;

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
