#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <numeric>
#include <unordered_map>
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
    }

    double AverageSpeed(int u, int v, double t_now) {
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
    int orders_count = 0;
    int conflicts_count = 0;
    CumulativeStatistic<double> speed;
    CumulativeStatistic<double> waiting_time;
    CumulativeStatistic<double> reverse_time;
    GraphEdgeStatistics edges;
};
