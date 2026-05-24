#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <numeric>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

template<class T>
class CumulativeStatistic {
public:
    void Add(const T& value) {
        values.push_back(value);
    }

    double Sum() const {
        return std::accumulate(values.begin(), values.end(), T(0));
    }

    double Average() const {
        return std::accumulate(values.begin(), values.end(), T(0)) / values.size();
    }

    const std::vector<T>& Values() const {
        return values;
    }

    bool Empty() const {
        return values.empty();
    }

private:
    std::vector<T> values;
};

// GraphEdgeStatistics aggregates per-edge passage observations and reports an
// effective travel speed for each edge.
//
// The reported speed is a *weighted harmonic mean* of observed passage speeds,
// optionally combined with a free-flow Bayesian prior. Formally, with t the
// query time, p ranging over stored passages on the edge, v_p the observed
// speed of passage p, and a_p = t - t^p_exit its age:
//
//                  w_0 + sum_p  w(a_p)
//   bar v(t)  =  -----------------------------,
//                w_0 / v_0  +  sum_p  w(a_p) / v_p
//
//   w(a) = exp(-a / tau)        if tau is finite and > 0
//        = 1                     otherwise (equal weights)
//
// Parameters (public fields, configurable at runtime):
//   - max_age        : hard cutoff. Passages with age > max_age are discarded
//                      from memory. Use std::numeric_limits<double>::infinity()
//                      to disable. Independent of tau (acts as a memory bound).
//   - tau            : EWMA decay time-constant. Infinite (default) means no
//                      decay, i.e. all retained samples are weighted equally.
//   - prior_weight   : w_0 -- weight of the free-flow Bayesian prior in units
//                      of "virtual passages". 0 (default) disables the prior.
//   - prior_speed    : v_0 -- the free-flow speed used by the prior. Ignored
//                      when prior_weight <= 0.
//
// Rationale for the harmonic mean: routing/ETA consumers actually want the
// expected travel time E[L/v], and E[L/v] != L / E[v] in general. The
// arithmetic mean of speeds systematically over-reports the effective speed
// when the distribution is bimodal (free-flow + congested), which is the
// regime where this estimator matters most.
//
// Rationale for time-decay weights: a FIFO size-bounded window has the
// pathological behavior that a stale fast sample stays at full weight until
// physically displaced by a new sample. On lightly trafficked edges this
// makes the reported speed appear to *worsen* in discrete jumps as old fast
// samples are evicted by newly arriving slow ones, even though the physical
// state of the edge has not changed. Exponential decay eliminates this
// artefact: old samples lose weight continuously with elapsed time, so the
// estimator reflects current conditions smoothly.
class GraphEdgeStatistics {
public:
    struct EdgePassage {
        double t_exit;
        double speed;
    };

    // Hard memory cutoff: passages older than max_age are discarded.
    double max_age = std::numeric_limits<double>::infinity();

    // EWMA time-constant. Set to infinity (default) for legacy equal weights.
    double tau = std::numeric_limits<double>::infinity();

    // Free-flow prior. Disabled by default (weight = 0).
    double prior_weight = 0.0;
    double prior_speed = 0.0;

    void Record(int u, int v, double t_enter, double t_exit,
                double length, double t_now) {
        if (length <= 0.0 || t_exit <= t_enter) {
            return;
        }
        auto& dq = data_[Key(u, v)];
        EvictOld(dq, t_now);
        dq.push_back({t_exit, length / (t_exit - t_enter)});
        ++version_;
    }

    // Monotonically-increasing change counter. Bumped on every mutation
    // (Record, Clear). Consumers can use it together with t_now to cheaply
    // detect whether AverageSpeed results computed earlier are still valid
    // and avoid recomputing the harmonic-mean reduction on the per-edge
    // hot path of a router. See StatAStarRouter.
    std::uint64_t Version() const { return version_; }

    // Test hook: number of times AverageSpeed has been invoked (counts both
    // cache misses and hits since the counter is incremented unconditionally
    // at entry). Useful for asserting that router-side caches actually
    // collapse repeated queries within a single simulation tick.
    mutable std::uint64_t query_count = 0;

    double AverageSpeed(int u, int v, double t_now) {
        ++query_count;
        auto it = data_.find(Key(u, v));
        const bool has_entry = (it != data_.end());
        if (has_entry) {
            EvictOld(it->second, t_now);
        }
        const bool has_data = has_entry && !it->second.empty();

        // No data and no prior -> legacy "no estimate" sentinel.
        if (!has_data && !(prior_weight > 0.0 && prior_speed > 0.0)) {
            return 0.0;
        }

        double sum_w = 0.0;
        double sum_w_over_v = 0.0;

        if (has_data) {
            const bool decay = std::isfinite(tau) && tau > 0.0;
            for (const auto& p : it->second) {
                if (!(p.speed > 0.0)) {
                    continue;
                }
                double w = 1.0;
                if (decay) {
                    const double age = t_now - p.t_exit;
                    // Negative ages (query before exit) clamp to 0 -> weight 1.
                    w = (age > 0.0) ? std::exp(-age / tau) : 1.0;
                }
                sum_w += w;
                sum_w_over_v += w / p.speed;
            }
        }

        if (prior_weight > 0.0 && prior_speed > 0.0) {
            sum_w += prior_weight;
            sum_w_over_v += prior_weight / prior_speed;
        }

        if (sum_w_over_v <= 0.0) {
            return 0.0;
        }
        return sum_w / sum_w_over_v;
    }

    bool Has(int u, int v) const {
        auto it = data_.find(Key(u, v));
        return it != data_.end() && !it->second.empty();
    }

    const std::deque<EdgePassage>* Window(int u, int v) const {
        auto it = data_.find(Key(u, v));
        if (it == data_.end()) {
            return nullptr;
        }
        return &it->second;
    }

    void Clear() {
        data_.clear();
        ++version_;
    }

private:
    static std::int64_t Key(int u, int v) {
        const int a = std::min(u, v);
        const int b = std::max(u, v);
        return (static_cast<std::int64_t>(static_cast<std::uint32_t>(a)) << 32)
             |  static_cast<std::int64_t>(static_cast<std::uint32_t>(b));
    }

    void EvictOld(std::deque<EdgePassage>& dq, double t_now) const {
        if (!(max_age < std::numeric_limits<double>::infinity())) {
            return;
        }
        const double cutoff = t_now - max_age;
        while (!dq.empty() && dq.front().t_exit < cutoff) {
            dq.pop_front();
        }
    }

    std::unordered_map<std::int64_t, std::deque<EdgePassage>> data_;
    std::uint64_t version_ = 0;
};

class Statistics {
public:
    static Statistics& Get() {
        static Statistics instance;
        return instance;
    }

    Statistics(const Statistics&) = delete;
    Statistics& operator=(const Statistics&) = delete;

private:
    Statistics() {}

public:
    GraphEdgeStatistics edges;

    int orders_count = 0;
    CumulativeStatistic<double> speed;
    CumulativeStatistic<double> order_time;

    CumulativeStatistic<double> waiting_time;

    int conflicts_count = 0;
    CumulativeStatistic<double> reverse_time;

    // ----- CCBS router instrumentation -----
    // Counters and distributions populated by CcbsRouter to validate the
    // tiered solve strategy (single-agent SIPP with frozen peers ->
    // joint CCBS -> A* last resort). All values are cumulative across
    // the lifetime of the process; tests reset Statistics::Get() between
    // runs as needed.
    //
    // - ccbs_singleagent_success: caller-only SIPP with frozen peers
    //   produced a valid path in the fast (or fallback) attempt.
    // - ccbs_joint_success      : full joint CCBS replan succeeded
    //   (used when the fast path failed but joint had room to manoeuvre).
    // - ccbs_fallback           : A* last-resort fallback was taken.
    // - ccbs_joint_task_size    : number of agents in each joint CCBS
    //   task (caller + relevant peers; idle/distant peers excluded).
    // - ccbs_solve_time_s       : wall-clock seconds spent inside the
    //   solver per GetRouteWithVertices() invocation (sum of all
    //   attempts including timeouts).
    int ccbs_singleagent_success = 0;
    int ccbs_joint_success = 0;
    int ccbs_fallback = 0;
    CumulativeStatistic<int> ccbs_joint_task_size;
    CumulativeStatistic<double> ccbs_solve_time_s;
};

// ---------------------------------------------------------------------------
// Metrics reporting
// ---------------------------------------------------------------------------
//
// MetricsReporter decouples *what* is printed at the end of a run from *where*
// it is printed and *which components were active*. Each metric is registered
// once together with:
//   - a label (for identification / test introspection),
//   - an optional condition over a ReportContext (which CLI components are
//     active), and
//   - a formatter producing the final string, or std::nullopt to suppress
//     the entry (e.g. when a CumulativeStatistic happens to be empty).
//
// The reporter holds no global state and is constructed locally per run.
// Adding a new metric is a single registration call in
// BuildDefaultMetricsReporter() -- no edits to main.cpp are required.

struct ReportContext {
    // Which router was selected on the command line. See demo/main.cpp.
    std::string router_kind = "astar";
    // Which narrow-edge resolver was selected on the command line.
    std::string resolver_kind = "none";
    // Whether visualization output was requested (--output).
    bool has_visualizer = false;

    bool UsesStatRouter() const { return router_kind == "stat"; }
    bool UsesCcbsRouter() const { return router_kind == "ccbs"; }
    bool UsesSemaphore()  const { return resolver_kind == "semaphore"; }
    bool UsesReverse()    const { return resolver_kind == "reverse"; }
};

class MetricsReporter {
public:
    using Condition = std::function<bool(const ReportContext&)>;
    // Returns the formatted "<label>: <value>" string, or std::nullopt when
    // the metric carries no data and should be omitted entirely.
    using Formatter = std::function<std::optional<std::string>()>;
    using Sink      = std::function<void(const std::string&)>;

    struct Entry {
        std::string label;
        Condition   cond;     // empty -> always active
        Formatter   format;
    };

    // Register an unconditional metric.
    MetricsReporter& Add(std::string label, Formatter format) {
        entries_.push_back({std::move(label), {}, std::move(format)});
        return *this;
    }

    // Register a metric that is only meaningful when `cond(ctx)` holds.
    MetricsReporter& AddIf(Condition cond, std::string label, Formatter format) {
        entries_.push_back({std::move(label), std::move(cond), std::move(format)});
        return *this;
    }

    // All registered entries, in registration order.
    const std::vector<Entry>& Entries() const { return entries_; }

    // Returns pointers to entries whose condition is satisfied by `ctx`,
    // preserving registration order. Does NOT invoke formatters and therefore
    // does NOT apply data-driven suppression -- this is the test-friendly
    // gating step.
    std::vector<const Entry*> SelectActive(const ReportContext& ctx) const {
        std::vector<const Entry*> out;
        out.reserve(entries_.size());
        for (const auto& e : entries_) {
            if (!e.cond || e.cond(ctx)) {
                out.push_back(&e);
            }
        }
        return out;
    }

    // Evaluate active entries and write each non-nullopt formatted line to
    // `sink`. Entries whose formatter returns std::nullopt are skipped
    // (e.g. empty CumulativeStatistic).
    void Print(const ReportContext& ctx, const Sink& sink) const {
        for (const Entry* e : SelectActive(ctx)) {
            auto line = e->format();
            if (line) {
                sink(*line);
            }
        }
    }

private:
    std::vector<Entry> entries_;
};

// Builds the default metrics reporter bound to `stats`. All metrics defined
// in Statistics are registered here; the registration is the single source of
// truth for which component owns which metric.
MetricsReporter BuildDefaultMetricsReporter(const Statistics& stats);
