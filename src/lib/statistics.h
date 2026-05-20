#pragma once

#include <algorithm>
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

class GraphEdgeStatistics {
public:
    static constexpr std::size_t kWindow = 15;

    struct EdgePassage {
        double t_exit;
        double speed;
    };

    double max_age = std::numeric_limits<double>::infinity();

    void Record(int u, int v, double t_enter, double t_exit,
                double length, double t_now) {
        if (length <= 0.0 || t_exit <= t_enter) {
            return;
        }
        auto& dq = data_[Key(u, v)];
        EvictOld(dq, t_now);
        dq.push_back({t_exit, length / (t_exit - t_enter)});
        while (dq.size() > kWindow) {
            dq.pop_front();
        }
    }

    double AverageSpeed(int u, int v, double t_now) {
        auto it = data_.find(Key(u, v));
        if (it == data_.end()) {
            return 0.0;
        }
        EvictOld(it->second, t_now);
        if (it->second.empty()) {
            return 0.0;
        }
        double sum = 0.0;
        for (const auto& p : it->second) {
            sum += p.speed;
        }
        return sum / static_cast<double>(it->second.size());
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
