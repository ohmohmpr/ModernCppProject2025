#pragma once
#include <chrono>
#include <iostream>

namespace voxel_map {
struct ScopedTimer {
  using Clock = std::chrono::high_resolution_clock;
  using TimePoint = std::chrono::time_point<Clock>;
  using Duration = std::chrono::duration<double>;

  TimePoint start_time;

  ScopedTimer() : start_time(Clock::now()) {}
  ~ScopedTimer() {
    const TimePoint end_time = Clock::now();
    const Duration duration = end_time - start_time;
    std::cout << "Execution took " << duration.count() << " seconds.\n";
  }

  // because we define a destructor, we have to define the rest of the 6 special
  // functions
  ScopedTimer(const ScopedTimer &) = default; // copy constructor
  ScopedTimer &
  operator=(const ScopedTimer &) = default;         // copy assignment operator
  ScopedTimer(ScopedTimer &&) = default;            // move constructor
  ScopedTimer &operator=(ScopedTimer &&) = default; // move assignment operator
};
} // namespace voxel_map