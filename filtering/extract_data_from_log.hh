#pragma once

#include "third_party/experiments/estimation/calibration/calibration_dataset.hh"
#include "third_party/experiments/estimation/time_point.hh"
#include "third_party/experiments/util/optional.hh"

#include <memory>

namespace ejf = estimation::jet_filter;
namespace jet {
namespace filtering {

struct TimeRange {
  estimation::TimePoint start = estimation::TimePoint::min();
  estimation::TimePoint end = estimation::TimePoint::max();
};

struct ExtractionConfiguration {
  bool use_imu = true;
  bool use_fiducial_detections = true;
};

estimation::CalibrationMeasurements extract_data_from_log(const std::string& log_path,
                                                          const TimeRange& time_range,
                                                          const ExtractionConfiguration& cfg = {});

class ImageStream {
 public:
  ImageStream(const std::string& log_path, const TimeRange& time_range);
  jcc::Optional<estimation::ImageMeasurement> next();

 private:
  class Nexter;
  std::shared_ptr<Nexter> nexter_;
  TimeRange range_;
};

}  // namespace filtering
}  // namespace jet