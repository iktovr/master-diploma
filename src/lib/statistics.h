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

namespace lib {

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

// Weighted harmonic mean of observed passage speeds with optional Bayesian prior.
// bar v(t) = (w_0 + sum_p w(a_p)) / (w_0/v_0 + sum_p w(a_p)/v_p), w(a) = exp(-a/tau) or 1.
// Parameters: max_age (hard cutoff), tau (EWMA decay, ∞=equal weights),
// prior_weight (w_0, 0=disabled), prior_speed (v_0).
class GraphEdgeStatistics {
public:
    struct EdgePassage {
        double t_exit;
        double speed;
    };

    double max_age = std::numeric_limits<double>::infinity();  // Hard memory cutoff
    double tau = std::numeric_limits<double>::infinity();      // EWMA decay (∞=equal weights)
    double prior_weight = 0.0;                                  // Free-flow prior weight (0=disabled)
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

    std::uint64_t Version() const { return version_; }
    mutable std::uint64_t query_count = 0;

    double AverageSpeed(int u, int v, double t_now) {
        ++query_count;
        auto it = data_.find(Key(u, v));
        const bool has_entry = (it != data_.end());
        if (has_entry) {
            EvictOld(it->second, t_now);
        }
        const bool has_data = has_entry && !it->second.empty();

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

    // ccbs_singleagent_success: caller-only SIPP with frozen peers succeeded
    // ccbs_joint_success: full joint CCBS replan succeeded
    // ccbs_fallback: A* last-resort fallback was taken
    // ccbs_joint_task_size: number of agents in joint CCBS task
    // ccbs_solve_time_s: wall-clock seconds per GetRouteWithVertices() invocation
    int ccbs_singleagent_success = 0;
    int ccbs_joint_success = 0;
    int ccbs_fallback = 0;
    CumulativeStatistic<int> ccbs_joint_task_size;
    CumulativeStatistic<double> ccbs_solve_time_s;
};

struct ReportContext {
    std::string router_kind = "astar";
    std::string resolver_kind = "none";
    bool has_visualizer = false;

    bool UsesStatRouter() const { return router_kind == "stat"; }
    bool UsesCcbsRouter() const { return router_kind == "ccbs"; }
    bool UsesSemaphore()  const { return resolver_kind == "semaphore"; }
    bool UsesReverse()    const { return resolver_kind == "reverse"; }
};

class MetricsReporter {
public:
    using Condition = std::function<bool(const ReportContext&)>;
    using Formatter = std::function<std::optional<std::string>()>;
    using Sink      = std::function<void(const std::string&)>;

    struct Entry {
        std::string label;
        Condition   cond;     // empty = always active
        Formatter   format;
    };

    MetricsReporter& Add(std::string label, Formatter format) {
        entries_.push_back({std::move(label), {}, std::move(format)});
        return *this;
    }

    MetricsReporter& AddIf(Condition cond, std::string label, Formatter format) {
        entries_.push_back({std::move(label), std::move(cond), std::move(format)});
        return *this;
    }

    const std::vector<Entry>& Entries() const { return entries_; }

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

MetricsReporter BuildDefaultMetricsReporter(const Statistics& stats);

}
