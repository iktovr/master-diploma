#include <cstdlib>
#include <filesystem>
#include <format>
#include <iostream>
#include <limits>
#include <string>

#include <CLI/CLI.hpp>

#include "lib/agent.h"
#include "lib/graph.h"
#include "lib/logging.h"
#include "lib/router.h"
#include "lib/statistics.h"
#include "lib/simulation.h"
#include "lib/visualizer.h"

namespace fs = std::filesystem;

int main(int argc, char **argv) {
    InitLogging();

    auto correct_path = CLI::Validator(
        [](std::string& input) {
            fs::path path(input);
            if (path.empty()) {
                return "Path is empty";
            }
            if (!path.is_absolute()) {
                path = fs::path(std::getenv("BUILD_WORKING_DIRECTORY")) / path;
            }
            input = path.string();
            return "";
        },
        "Make path absolute with actual working dir if it is not absolute already",
        "Correct path"
    );

    CLI::App app{"demo"};
    argv = app.ensure_utf8(argv);

    fs::path graph_path;
    double duration = 5;
    double step = 0.1;
    int agents_count = 10;
    double max_speed = 2.2;
    fs::path output_dir;
    int frame_size = 0;
    double scale = 1.0;
    double vis_step = 0.1;
    fs::path animation_path;
    int animation_framerate = 5;
    int basepoints_limit = -1;
    std::string router_kind = "astar";
    double max_age = std::numeric_limits<double>::infinity();

    app.add_option("-g, --graph", graph_path, "Path to file with graph description")
        ->required()->transform(correct_path)->check(CLI::ExistingFile);
    app.add_option("-d, --duration", duration, "Duration of simulation")
        ->check(CLI::PositiveNumber);
    app.add_option("-t, --tick", step, "Duration of simulated step")
        ->check(CLI::PositiveNumber);
    app.add_option("-a, --agents", agents_count, "Number of agents")
        ->check(CLI::PositiveNumber);
    app.add_option("-s, --speed", max_speed, "Maximum agent speed");
    app.add_option("--max-age", max_age, "Maximum age graph edge statistics")
        ->check(CLI::PositiveNumber);

    app.add_option("-o, --output", output_dir, "Directory for visualizations")
        ->transform(correct_path)->check(CLI::ExistingDirectory);
    app.add_option("-f, --frame", frame_size, "Maximum frame size in pixels (0 = auto, scaled to graph density)")
        ->check(CLI::NonNegativeNumber);
    app.add_option("--scale", scale, "Additional objects scale")
        ->check(CLI::PositiveNumber);
    app.add_option("-v, --visualize-step", vis_step, "Period of visualizations")
        ->check(CLI::PositiveNumber);
    app.add_option("-b, --basepoints", basepoints_limit, "Number of base points to keep from the GeoJSON map (-1 = all)")
        ->check(CLI::Range(-1, std::numeric_limits<int>::max()));
    app.add_option("-r, --router", router_kind, "Router kind: astar (length) or stat (travel time from edge statistics)")
        ->check(CLI::IsMember({"astar", "stat"}));

    app.add_option("--animate", animation_path, "Convert visualization frames to animation using ffmpeg")
        ->transform(correct_path);
    app.add_option("--framerate", animation_framerate, "Animation framerate");

    CLI11_PARSE(app, argc, argv);

    auto g = std::make_shared<const Graph>(
        (graph_path.extension() == ".geojson")
            ? Graph::LoadFromGeoJsonFile(graph_path, basepoints_limit)
            : Graph::LoadFromFile(graph_path));
    std::optional<Visualizer> vis;
    if (!output_dir.empty()) {
        if (!fs::exists(output_dir)) {
            fs::create_directories(output_dir);
        } else {
            for (const auto& dir_entry : fs::directory_iterator(output_dir)) {
                if (dir_entry.is_regular_file() && dir_entry.path().filename().string().starts_with("frame")) {
                    fs::remove(dir_entry);
                }
            }
        }
        const int effective_frame_size = (frame_size > 0) ? frame_size : ComputeAutoFrameSize(*g);
        vis.emplace(g->Width(), g->Height(), g->Centroid(), effective_frame_size, scale, output_dir, max_speed);
    }
    Statistics::Get().edges.max_age = max_age;

    Agents agents(agents_count, Agent(0, 0, 0));
    std::shared_ptr<const IRouter> router;
    if (router_kind == "stat") {
        router = std::make_shared<StatAStarRouter>(g, &Statistics::Get().edges, max_speed);
    } else {
        router = std::make_shared<AStarRouter>(g);
    }
    Simulation sim(max_speed, std::move(agents), g, router, vis);
    sim.Simulate(duration, step, vis_step);

    LOG_INFO("Number of orders: {}", Statistics::Get().orders_count);
    if (!Statistics::Get().waiting_time.Empty()) {
        LOG_INFO("Average waiting time: {}", Statistics::Get().waiting_time.Average());
    }
    if (!Statistics::Get().speed.Empty()) {
        LOG_INFO("Average speed: {}", Statistics::Get().speed.Average());
    }

    if (vis && !animation_path.empty()) {
        std::cout.flush();
        const std::string command = std::format("ffmpeg -f image2 -framerate {} -i {}/frame_%04d.png -y {}", animation_framerate, output_dir.c_str(), animation_path.c_str());
        std::system(command.c_str());
        std::cerr << command << std::endl;
    }
}
