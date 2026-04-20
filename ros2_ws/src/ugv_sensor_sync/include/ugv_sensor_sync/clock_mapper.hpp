#pragma once

#include <cstdint>
#include <deque>
#include <mutex>
#include <utility>

namespace ugv_sensor_sync
{

class ClockMapper
{
public:
  explicit ClockMapper(std::size_t max_samples = 200);

  void addSample(uint64_t esp32_time_us, int64_t ros_time_ns);

  bool ready() const;

  int64_t mapEsp32UsToRosNs(uint64_t esp32_time_us) const;

  std::pair<double, double> getFit() const;

private:
  void recomputeFit();

  std::size_t max_samples_;
  mutable std::mutex mutex_;
  std::deque<std::pair<uint64_t, int64_t>> samples_;

  double a_;
  double b_;
  bool ready_;
};

}  // namespace ugv_sensor_sync