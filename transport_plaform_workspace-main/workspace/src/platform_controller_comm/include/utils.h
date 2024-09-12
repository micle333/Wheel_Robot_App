#pragma once
#include <chrono>
#include <thread>

namespace utils {
    void delay(long long ms) {
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    }

    long long unsigned get_us() {
        return std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::high_resolution_clock::now().time_since_epoch()
        ).count();
    }
}

