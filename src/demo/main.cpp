#include <cstdlib>
#include <filesystem>
#include <format>
#include <iostream>
#include <limits>
#include <string>

#include <CLI/CLI.hpp>

#include "lib/agent.h"
#include "lib/ccbs_router.h"
#include "lib/graph.h"
#include "lib/logging.h"
#include "lib/router.h"
#include "lib/statistics.h"
#include "lib/simulation.h"
#include "lib/visualizer.h"

using namespace lib;
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
                auto working_dir = std::getenv("BUILD_WORKING_DIRECTORY");
                path = fs::path(working_dir ? working_dir : ".") / path;
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
    std::string resolver_kind = "none";
    double max_age = 300;
    double ewma_tau = 60;
    double prior_weight = 1.0;
    bool draw_graph = true;
    bool draw_agents = true;
    bool draw_statistics = false;
    bool draw_points = true;
    bool draw_narrow_edges = true;
    bool draw_overlay_stats = false;

    app.add_option("-g, --graph", graph_path, "Path to file with graph description")
        ->required()->transform(correct_path)->check(CLI::ExistingFile);
    app.add_option("-d, --duration", duration, "Duration of simulation")
        ->check(CLI::PositiveNumber);
    app.add_option("-t, --tick", step, "Duration of simulated step")
        ->check(CLI::PositiveNumber);
    app.add_option("-a, --agents", agents_count, "Number of agents")
        ->check(CLI::PositiveNumber);
    app.add_option("-s, --speed", max_speed, "Maximum agent speed");
    app.add_option("--max-age", max_age,
                   "Hard memory cutoff (seconds) for graph edge statistics; Default: infinity.")
        ->check(CLI::PositiveNumber);
    app.add_option("--ewma-tau", ewma_tau,
                   "EWMA decay time-constant (seconds) for graph edge speed estimate")
        ->check(CLI::PositiveNumber);
    app.add_option("--prior-weight", prior_weight,
                   "Weight of the free-flow Bayesian prior in the edge speed estimate.")
        ->check(CLI::NonNegativeNumber);

    app.add_option("-o, --output", output_dir, "Directory for visualizations")
        ->transform(correct_path)->check(CLI::ExistingDirectory);
    app.add_option("-f, --frame", frame_size, "Maximum frame size in pixels")
        ->check(CLI::NonNegativeNumber);
    app.add_option("--scale", scale, "Additional objects scale")
        ->check(CLI::PositiveNumber);
    app.add_option("-v, --visualize-step", vis_step, "Period of visualizations")
        ->check(CLI::PositiveNumber);
    app.add_option("-b, --basepoints", basepoints_limit, "Number of base points to keep from the GeoJSON map (-1 = all)")
        ->check(CLI::Range(-1, std::numeric_limits<int>::max()));
    app.add_option("-r, --router", router_kind,
                   "Router kind")
        ->check(CLI::IsMember({"astar", "stat", "ccbs"}));
    app.add_option("--resolver", resolver_kind, "Narrow-edge conflict resolver")
        ->check(CLI::IsMember({"none", "semaphore", "reverse"}));
    app.add_flag("--draw-graph,!--no-draw-graph", draw_graph, "Draw graph structure (default: true)");
    app.add_flag("--draw-agents,!--no-draw-agents", draw_agents, "Draw agents (default: true)");
    app.add_flag("--draw-statistics,!--no-draw-statistics", draw_statistics, "Draw graph statistics (default: false)");
    app.add_flag("--draw-points,!--no-draw-points", draw_points, "Draw graph points (default: true)");
    app.add_flag("--draw-narrow-edges,!--no-draw-narrow-edges", draw_narrow_edges, "Draw narrow edges (default: true)");
    app.add_flag("--draw-overlay-stats,!--no-draw-overlay-stats", draw_overlay_stats, "Draw aggregate statistics overlay in upper-left corner (default: false)");

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
        vis.emplace(g->Width(), g->Height(), g->Centroid(), effective_frame_size, scale, output_dir, max_speed,
                   draw_graph, draw_agents, draw_statistics, draw_points, draw_narrow_edges, draw_overlay_stats);
    }
    Statistics::Get().edges.max_age = max_age;
    Statistics::Get().edges.tau = ewma_tau;
    Statistics::Get().edges.prior_weight = prior_weight;
    Statistics::Get().edges.prior_speed = max_speed;

    Agents agents(agents_count, Agent(0, 0, 0));
    if (router_kind == "ccbs" && resolver_kind != "none") {
        std::cerr << "Error: --router=ccbs is incompatible with "
                     "--resolver=" << resolver_kind
                  << "; use --resolver=none." << std::endl;
        return EXIT_FAILURE;
    }
    std::shared_ptr<const IRouter> router;
    std::shared_ptr<CcbsRouter> ccbs_router;
    if (router_kind == "stat") {
        router = std::make_shared<StatAStarRouter>(g, &Statistics::Get().edges, max_speed);
    } else if (router_kind == "ccbs") {
        ccbs_router = std::make_shared<CcbsRouter>(g, nullptr, max_speed);
        router = ccbs_router;
    } else {
        router = std::make_shared<AStarRouter>(g);
    }
    ResolverKind rk = ResolverKind::none;
    if (resolver_kind == "semaphore") {
        rk = ResolverKind::semaphore;
    } else if (resolver_kind == "reverse") {
        rk = ResolverKind::reverse;
    }
    Simulation sim(max_speed, std::move(agents), g, router, vis, rk);
    if (ccbs_router) {
        ccbs_router->SetAgents(&sim.agents);
    }
    sim.Simulate(duration, step, vis_step);

    ReportContext report_ctx{
        .router_kind   = router_kind,
        .resolver_kind = resolver_kind,
        .has_visualizer = vis.has_value(),
    };
    MetricsReporter reporter = BuildDefaultMetricsReporter(Statistics::Get());
    reporter.Print(report_ctx, [](const std::string& line) {
        LOG_INFO("{}", line);
    });

    if (vis && !animation_path.empty()) {
        std::cout.flush();
        const std::string command = std::format("ffmpeg -f image2 -framerate {} -i {}/frame_%04d.png -y {}", animation_framerate, output_dir.c_str(), animation_path.c_str());
        std::system(command.c_str());
        std::cerr << command << std::endl;
    }
}
