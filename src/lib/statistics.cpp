#include "statistics.h"

#include <format>
#include <optional>
#include <string>

namespace {

template <class T>
MetricsReporter::Formatter ScalarFormatter(std::string label, const T& ref) {
    return [label = std::move(label), &ref]() -> std::optional<std::string> {
        return std::format("{}: {}", label, ref);
    };
}

template <class T>
MetricsReporter::Formatter AverageFormatter(std::string label,
                                            const CumulativeStatistic<T>& stat) {
    return [label = std::move(label), &stat]() -> std::optional<std::string> {
        if (stat.Empty()) {
            return std::nullopt;
        }
        return std::format("{}: {}", label, stat.Average());
    };
}

template <class T>
MetricsReporter::Formatter SumFormatter(std::string label,
                                        const CumulativeStatistic<T>& stat) {
    return [label = std::move(label), &stat]() -> std::optional<std::string> {
        if (stat.Empty()) {
            return std::nullopt;
        }
        return std::format("{}: {}", label, stat.Sum());
    };
}

}  // namespace

MetricsReporter BuildDefaultMetricsReporter(const Statistics& stats) {
    MetricsReporter r;

    // --- Always-on simulation-wide metrics -----------------------------------
    r.Add("Number of orders",
          ScalarFormatter("Number of orders", stats.orders_count));
    r.Add("Average speed",
          AverageFormatter("Average speed", stats.speed));

    // --- Resolver-specific metrics ------------------------------------------
    r.AddIf([](const ReportContext& ctx) { return ctx.UsesSemaphore() || ctx.UsesReverse(); },
            "Average waiting time",
            AverageFormatter("Average waiting time", stats.waiting_time));
    r.AddIf([](const ReportContext& ctx) { return ctx.UsesReverse(); },
            "Average reverse time",
            AverageFormatter("Average reverse time", stats.reverse_time));
    r.AddIf([](const ReportContext& ctx) { return ctx.UsesReverse(); },
            "Number of conflicts",
            ScalarFormatter("Number of conflicts", stats.conflicts_count));

    // --- CCBS router instrumentation ----------------------------------------
    auto uses_ccbs = [](const ReportContext& ctx) { return ctx.UsesCcbsRouter(); };
    r.AddIf(uses_ccbs, "CCBS single-agent successes",
            ScalarFormatter("CCBS single-agent successes",
                            stats.ccbs_singleagent_success));
    r.AddIf(uses_ccbs, "CCBS joint successes",
            ScalarFormatter("CCBS joint successes", stats.ccbs_joint_success));
    r.AddIf(uses_ccbs, "CCBS fallbacks",
            ScalarFormatter("CCBS fallbacks", stats.ccbs_fallback));
    r.AddIf(uses_ccbs, "CCBS average joint task size",
            AverageFormatter("CCBS average joint task size",
                             stats.ccbs_joint_task_size));
    r.AddIf(uses_ccbs, "CCBS total solve time (s)",
            SumFormatter("CCBS total solve time (s)", stats.ccbs_solve_time_s));
    r.AddIf(uses_ccbs, "CCBS average solve time (s)",
            AverageFormatter("CCBS average solve time (s)",
                             stats.ccbs_solve_time_s));

    return r;
}
