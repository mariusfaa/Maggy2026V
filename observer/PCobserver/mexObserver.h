#pragma once

#include "observer.h"
#include <map>
#include <stdexcept>
#include <cstdint>

// Shared across all MEX files that link this TU
using filterPtr = std::unique_ptr<KalmanFilter>;

class ObserverRegistry {
public:
    static ObserverRegistry& instance() {
        static ObserverRegistry reg;
        return reg;
    }

    uint64_t store(filterPtr ptr) {
        uint64_t handle = nextHandle_++;
        map_[handle] = std::move(ptr);
        return handle;
    }

    KalmanFilter& get(uint64_t handle) {
        auto it = map_.find(handle);
        if (it == map_.end())
            throw std::runtime_error("Invalid observer handle");
        return *it->second;
    }

    void destroy(uint64_t handle) {
        if (!map_.erase(handle))
            throw std::runtime_error("Invalid observer handle");
    }

private:
    std::map<uint64_t, filterPtr> map_;
    uint64_t nextHandle_ = 1;
};
