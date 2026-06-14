#include <cassert>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <CLI/CLI.hpp>
#include <nlohmann/json.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "lib/geometry.h"
#include "lib/graph.h"
#include "lib/visualizer.h"

using namespace lib;
namespace fs = std::filesystem;
using json = nlohmann::json;

namespace {

std::vector<Point> LoadLocationPolygon(const fs::path& path, const Graph& graph) {
    std::ifstream file(path);
    assert(file.is_open());
    const json root = json::parse(file);

    const json* geom = nullptr;
    const std::string type = root.at("type").get<std::string>();
    if (type == "Polygon") {
        geom = &root;
    } else if (type == "Feature") {
        geom = &root.at("geometry");
    } else if (type == "FeatureCollection") {
        for (const auto& feature : root.at("features")) {
            if (feature.at("geometry").at("type").get<std::string>() == "Polygon") {
                geom = &feature.at("geometry");
                break;
            }
        }
    }
    assert(geom != nullptr);
    assert(geom->at("type").get<std::string>() == "Polygon");

    std::vector<Point> polygon;
    for (const auto& c : geom->at("coordinates")[0]) {
        polygon.push_back(graph.ProjectLonLat(c[0].get<double>(), c[1].get<double>()));
    }
    return polygon;
}

class LocationVisualizer : public Visualizer {
public:
    using Visualizer::Visualizer;

    void DrawLocation(const std::vector<Point>& polygon, const cv::Scalar& color) {
        std::vector<cv::Point> pts;
        pts.reserve(polygon.size());
        for (const auto& p : polygon) {
            pts.push_back(ToPixels(p));
        }
        std::vector<std::vector<cv::Point>> polys{pts};
        cv::polylines(img, polys, true, color, std::max(1, edge_thickness_px * 3), cv::LINE_AA);
    }

    void SaveToFile(const fs::path& path) {
        cv::imwrite(path.string(), img);
    }
};

}  // namespace

int main(int argc, char** argv) {
    auto correct_path = CLI::Validator(
        [](std::string& input) {
            fs::path path(input);
            if (path.empty()) {
                return "Path is empty";
            }
            if (!path.is_absolute()) {
                auto working_dir = std::getenv("BUILD_WORKING_DIRECTORY");
                path = fs::path(working_dir ? working_dir : ".") / path;
            }
            input = path.string();
            return "";
        },
        "Make path absolute with actual working dir if it is not absolute already",
        "Correct path");

    CLI::App app{"draw_locations"};
    argv = app.ensure_utf8(argv);

    fs::path graph_path;
    std::vector<fs::path> location_paths;
    fs::path output_path;
    int frame_size = 0;
    double scale = 1.0;

    app.add_option("-g,--graph", graph_path, "Path to GeoJSON file with graph description")
        ->required()->transform(correct_path)->check(CLI::ExistingFile);
    app.add_option("-l,--location", location_paths, "Paths to three GeoJSON location files")
        ->required()->expected(3)->transform(correct_path)->check(CLI::ExistingFile);
    app.add_option("-o,--output", output_path, "Path to output image file")
        ->required()->transform(correct_path);
    app.add_option("-f,--frame", frame_size, "Maximum frame size in pixels")
        ->check(CLI::NonNegativeNumber);
    app.add_option("--scale", scale, "Additional objects scale")
        ->check(CLI::PositiveNumber);

    CLI11_PARSE(app, argc, argv);

    const Graph graph = Graph::LoadFromGeoJsonFile(graph_path);

    std::vector<std::vector<Point>> locations;
    for (const auto& path : location_paths) {
        locations.push_back(LoadLocationPolygon(path, graph));
    }

    fs::path output_dir = output_path.parent_path();
    if (output_dir.empty()) {
        output_dir = ".";
    }
    if (!fs::exists(output_dir)) {
        fs::create_directories(output_dir);
    }

    const int effective_frame_size = (frame_size > 0) ? frame_size : ComputeAutoFrameSize(graph);
    LocationVisualizer vis(graph.Width(), graph.Height(), graph.Centroid(), effective_frame_size,
                           scale, output_dir.string(), 0.0,
                           true, false, false, true, true, false);

    const std::vector<cv::Scalar> colors = {
        Color("#4daf4a"),
        Color("#377eb8"),
        Color("#e41a1c"),
    };

    vis.DrawGraph(graph);
    for (std::size_t i = 0; i < locations.size(); ++i) {
        vis.DrawLocation(locations[i], colors[i % colors.size()]);
    }

    vis.SaveToFile(output_path);

    std::cout << "Saved " << output_path.string() << std::endl;
    return EXIT_SUCCESS;
}
