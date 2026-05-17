#pragma once

#include <vector>
#include <numeric>

/* 
число заказов
средняя скорость
среднее время ожидания
*/

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
    CumulativeStatistic<double> speed;
    CumulativeStatistic<double> waiting_time;
};