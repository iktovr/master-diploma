#include <filesystem>
#include <string>

#include <CLI/CLI.hpp>

#include "lib/agent.h"
#include "lib/graph.h"
#include "lib/simulation.h"
#include "lib/visualizer.h"

namespace fs = std::filesystem;

int main(int argc, char **argv) {
    CLI::App app{"demo"};
    argv = app.ensure_utf8(argv);

    // fs::path output_dir, graph_path;
    // int max_frame_size = 1000;
    // app.add_option('-o, --output', output_dir, "Directory for visualizations");
    // app.add_option('-g, --graph', graph_path, "Path to file with graph description");

    Graph g = Graph::LoadFromFile("/workspaces/master_diploma/src/demo/graph.txt");
    Visualizer vis(g.Width() + 0.5, g.Height() + 0.5, g.Centroid(), 1000, "/workspaces/master_diploma/src/demo/out");

    Agents agents(10, Agent(-1, 0, 0));
    Simulation sim(std::move(agents), std::move(g), vis);
    sim.Simulate(5, 0.1, 0.1);
}
