#include "lib/statistics.h"

#include <algorithm>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace {

// Returns the list of labels selected by the reporter for the given context,
// in registration order.
std::vector<std::string> ActiveLabels(const MetricsReporter& r,
                                      const ReportContext& ctx) {
    std::vector<std::string> out;
    for (const auto* e : r.SelectActive(ctx)) {
        out.push_back(e->label);
    }
    return out;
}

// Runs Print() and collects emitted lines.
std::vector<std::string> Emitted(const MetricsReporter& r,
                                 const ReportContext& ctx) {
    std::vector<std::string> lines;
    r.Print(ctx, [&](const std::string& line) { lines.push_back(line); });
    return lines;
}

bool Contains(const std::vector<std::string>& v, const std::string& needle) {
    return std::any_of(v.begin(), v.end(),
                       [&](const std::string& s) { return s.find(needle) != std::string::npos; });
}

}  // namespace

// ---------------------------------------------------------------------------
// Gating: a metric registered with AddIf must appear iff the predicate holds.
// ---------------------------------------------------------------------------
TEST(MetricsReporter, AddIfRespectsCondition) {
    MetricsReporter r;
    r.Add("always", []() -> std::optional<std::string> { return "always: 1"; });
    r.AddIf([](const ReportContext& c) { return c.UsesCcbsRouter(); },
            "ccbs-only",
            []() -> std::optional<std::string> { return "ccbs-only: 1"; });

    ReportContext astar{.router_kind = "astar"};
    ReportContext ccbs {.router_kind = "ccbs"};

    EXPECT_EQ(ActiveLabels(r, astar), (std::vector<std::string>{"always"}));
    EXPECT_EQ(ActiveLabels(r, ccbs),
              (std::vector<std::string>{"always", "ccbs-only"}));
}

// ---------------------------------------------------------------------------
// Empty-data suppression: formatter returning std::nullopt is omitted from
// Print() output but its entry is still considered "active" by SelectActive.
// ---------------------------------------------------------------------------
TEST(MetricsReporter, NulloptFormatterSuppressesLine) {
    MetricsReporter r;
    r.Add("present", []() -> std::optional<std::string> { return "present: 42"; });
    r.Add("absent",  []() -> std::optional<std::string> { return std::nullopt; });

    ReportContext ctx{};
    EXPECT_EQ(ActiveLabels(r, ctx),
              (std::vector<std::string>{"present", "absent"}));
    EXPECT_EQ(Emitted(r, ctx),
              (std::vector<std::string>{"present: 42"}));
}

// ---------------------------------------------------------------------------
// Registration order is preserved across both SelectActive and Print.
// ---------------------------------------------------------------------------
TEST(MetricsReporter, PreservesRegistrationOrder) {
    MetricsReporter r;
    r.Add("a", []() -> std::optional<std::string> { return "a"; });
    r.Add("b", []() -> std::optional<std::string> { return "b"; });
    r.Add("c", []() -> std::optional<std::string> { return "c"; });

    ReportContext ctx{};
    EXPECT_EQ(Emitted(r, ctx), (std::vector<std::string>{"a", "b", "c"}));
}

// ---------------------------------------------------------------------------
// Default reporter: AStar + no resolver should only show always-on metrics
// (no waiting/reverse/CCBS lines), and CumulativeStatistics that have no
// samples are suppressed.
// ---------------------------------------------------------------------------
TEST(BuildDefaultMetricsReporter, AstarNoneOnlyAlwaysOnLines) {
    Statistics& stats = Statistics::Get();
    stats.orders_count = 3;
    stats.conflicts_count = 1;
    stats.speed = CumulativeStatistic<double>{};  // empty
    stats.waiting_time = CumulativeStatistic<double>{};
    stats.reverse_time = CumulativeStatistic<double>{};
    stats.ccbs_singleagent_success = 0;
    stats.ccbs_joint_success = 0;
    stats.ccbs_fallback = 0;
    stats.ccbs_joint_task_size = CumulativeStatistic<int>{};
    stats.ccbs_solve_time_s = CumulativeStatistic<double>{};

    MetricsReporter r = BuildDefaultMetricsReporter(stats);
    ReportContext ctx{.router_kind = "astar", .resolver_kind = "none"};
    auto lines = Emitted(r, ctx);

    EXPECT_TRUE(Contains(lines, "Number of orders: 3"));
    // Conflicts line is only shown for reverse resolver, not for none resolver
    EXPECT_FALSE(Contains(lines, "Number of conflicts"));
    // Empty 'speed' is suppressed.
    EXPECT_FALSE(Contains(lines, "Average speed"));
    // Resolver-specific lines must not appear.
    EXPECT_FALSE(Contains(lines, "waiting time"));
    EXPECT_FALSE(Contains(lines, "reverse time"));
    // CCBS-specific lines must not appear.
    EXPECT_FALSE(Contains(lines, "CCBS"));
}

// ---------------------------------------------------------------------------
// Resolver gating: semaphore-only metric appears under --resolver=semaphore,
// reverse-only metric appears under --resolver=reverse; never both.
// ---------------------------------------------------------------------------
TEST(BuildDefaultMetricsReporter, ResolverGating) {
    Statistics& stats = Statistics::Get();
    stats.orders_count = 0;
    stats.conflicts_count = 0;
    stats.speed = CumulativeStatistic<double>{};
    stats.waiting_time = CumulativeStatistic<double>{};
    stats.waiting_time.Add(1.5);
    stats.reverse_time = CumulativeStatistic<double>{};
    stats.reverse_time.Add(2.5);
    stats.ccbs_joint_task_size = CumulativeStatistic<int>{};
    stats.ccbs_solve_time_s = CumulativeStatistic<double>{};

    MetricsReporter r = BuildDefaultMetricsReporter(stats);

    auto sem = Emitted(r, ReportContext{.resolver_kind = "semaphore"});
    EXPECT_TRUE(Contains(sem, "Average waiting time"));
    EXPECT_FALSE(Contains(sem, "Average reverse time"));

    auto rev = Emitted(r, ReportContext{.resolver_kind = "reverse"});
    EXPECT_TRUE(Contains(rev, "Average waiting time"));
    EXPECT_TRUE(Contains(rev, "Average reverse time"));

    auto none = Emitted(r, ReportContext{.resolver_kind = "none"});
    EXPECT_FALSE(Contains(none, "Average waiting time"));
    EXPECT_FALSE(Contains(none, "Average reverse time"));
}

// ---------------------------------------------------------------------------
// CCBS gating + empty-stat suppression for CCBS distributions.
// ---------------------------------------------------------------------------
TEST(BuildDefaultMetricsReporter, CcbsGating) {
    Statistics& stats = Statistics::Get();
    stats.orders_count = 0;
    stats.conflicts_count = 0;
    stats.speed = CumulativeStatistic<double>{};
    stats.waiting_time = CumulativeStatistic<double>{};
    stats.reverse_time = CumulativeStatistic<double>{};
    stats.ccbs_singleagent_success = 7;
    stats.ccbs_joint_success = 2;
    stats.ccbs_fallback = 1;
    stats.ccbs_joint_task_size = CumulativeStatistic<int>{};  // empty -> suppressed
    stats.ccbs_solve_time_s = CumulativeStatistic<double>{};
    stats.ccbs_solve_time_s.Add(0.25);
    stats.ccbs_solve_time_s.Add(0.75);

    MetricsReporter r = BuildDefaultMetricsReporter(stats);

    auto astar = Emitted(r, ReportContext{.router_kind = "astar"});
    EXPECT_FALSE(Contains(astar, "CCBS"));

    auto ccbs = Emitted(r, ReportContext{.router_kind = "ccbs"});
    EXPECT_TRUE(Contains(ccbs, "CCBS single-agent successes: 7"));
    EXPECT_TRUE(Contains(ccbs, "CCBS joint successes: 2"));
    EXPECT_TRUE(Contains(ccbs, "CCBS fallbacks: 1"));
    // Empty distribution must be suppressed.
    EXPECT_FALSE(Contains(ccbs, "joint task size"));
    // Non-empty distribution must appear.
    EXPECT_TRUE(Contains(ccbs, "CCBS total solve time"));
    EXPECT_TRUE(Contains(ccbs, "CCBS average solve time"));
}
