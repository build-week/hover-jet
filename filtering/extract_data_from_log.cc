#include "filtering/extract_data_from_log.hh"

#include "camera/camera_image_message.hh"
#include "embedded/imu_driver/imu_message.hh"
#include "filtering/to_time_point.hh"
#include "infrastructure/logging/log_reader.hh"
#include "vision/fiducial_detection_message.hh"

//%deps(log)
#include "third_party/experiments/logging/log.hh"
#include "third_party/experiments/out.hh"

namespace jet {
namespace filtering {

namespace {
void add_imu(const ImuMessage& msg,
             const TimeRange& range,
             Out<estimation::calibration::CalibrationMeasurements> meas) {
  const auto time_of_validity = to_time_point(msg.timestamp);

  if ((time_of_validity < range.start) || (time_of_validity > range.end)) {
    return;
  }

  const int imu_id = msg.imu_id_leading_byte;
  // Today, it is redundancy day today
  meas->imu_cal[imu_id].imu_id = imu_id;
  {
    const jcc::Vec3 accel_mpss(msg.accel_mpss_x, msg.accel_mpss_y, msg.accel_mpss_z);
    ejf::AccelMeasurement accel_meas;
    accel_meas.observed_acceleration = accel_mpss;
    meas->imu_cal[imu_id].accel_meas.push_back({accel_meas, time_of_validity});
  }
  {
    const jcc::Vec3 gyro_radps(msg.gyro_radps_x, msg.gyro_radps_y, msg.gyro_radps_z);
    ejf::GyroMeasurement gyro_meas;
    gyro_meas.observed_w = gyro_radps;
    meas->imu_cal[imu_id].gyro_meas.push_back({gyro_meas, time_of_validity});
  }

  {
    const jcc::Vec3 mag_utesla(msg.mag_utesla_x, msg.mag_utesla_y, msg.mag_utesla_z);
    estimation::MagnetometerMeasurement mag_meas;
    mag_meas.observed_bfield = mag_utesla;
    meas->imu_cal[imu_id].mag_meas.push_back({mag_meas, time_of_validity});
  }
}

void add_fiducial(const FiducialDetectionMessage& msg,
                  const TimeRange& range,
                  Out<estimation::calibration::CalibrationMeasurements> meas) {
  const auto time_of_validity = to_time_point(msg.timestamp);

  if ((time_of_validity < range.start) || (time_of_validity > range.end)) {
    return;
  }

  const ejf::FiducialMeasurement fiducial_meas{
      .fiducial_id = 1,                                     //
      .T_fiducial_from_camera = msg.fiducial_from_camera()  //
  };

  meas->fiducial_meas.push_back({fiducial_meas, time_of_validity});
}

}  // namespace

estimation::calibration::CalibrationMeasurements extract_data_from_log(const std::string& log_path,
                                                                       const TimeRange& time_range,
                                                                       const ExtractionConfiguration& cfg) {
  estimation::calibration::CalibrationMeasurements meas;

  const std::vector<std::string> channel_names = {"imu", "fiducial_detection_channel", "camera_image_channel", "imu_1",
                                                  "imu_2"};

  jet::LogReader reader(log_path, channel_names);

  const auto channels_list = reader.get_available_channels();
  const bool one_imu_present = std::find(channels_list.begin(), channels_list.end(), "imu") != channels_list.end();

  const std::vector<std::string> single_imu({{"imu"}});
  const std::vector<std::string> both_imu({{"imu_1"}, {"imu_2"}});

  const std::vector<std::string> imu_channels = one_imu_present ? single_imu : both_imu;

  // const bool two_imus = !one_imu_present;
  // Read until we have read all IMU messages
  bool got_anything = true;
  while (got_anything) {
    got_anything = false;
    if (cfg.use_imu) {
      for (const auto& imu_channel : imu_channels) {
        ImuMessage imu_msg;
        if (reader.read_next_message(imu_channel, imu_msg)) {
          add_imu(imu_msg, time_range, out(meas));
          got_anything = true;
        } else {
          break;
        }
      }
    }

    if (cfg.use_fiducial_detections) {
      FiducialDetectionMessage fiducial_msg;
      if (reader.read_next_message("fiducial_detection_channel", fiducial_msg)) {
        add_fiducial(fiducial_msg, time_range, out(meas));
        got_anything = true;
      }
    }
  }

  for (auto& field : meas.imu_cal) {
    field.second.set_first(meas.first());
    field.second.set_last(meas.last());
  }

  return meas;
}

class ImageStream::Nexter {
 public:
  Nexter(const std::string& log_path) {
    const std::vector<std::string> ch_nms = {"camera_image_channel"};
    reader_ = std::make_shared<jet::LogReader>(log_path, ch_nms);
  }

  bool get_next_image(Out<CameraImageMessage> msg) {
    return reader_->read_next_message("camera_image_channel", *msg);
  }

 private:
  std::shared_ptr<jet::LogReader> reader_;
};

ImageStream::ImageStream(const std::string& log_path, const TimeRange& time_range) : range_(time_range) {
  nexter_ = std::make_shared<Nexter>(log_path);
}

jcc::Optional<estimation::ImageMeasurement> ImageStream::next() {
  CameraImageMessage img_msg;
  if (!nexter_->get_next_image(out(img_msg))) {
    return {};
  }

  const auto time_of_validity = to_time_point(img_msg.timestamp);

  if ((time_of_validity < range_.start) || (time_of_validity > range_.end)) {
    return {};
  }

  const cv::Mat image = get_image_mat(std::move(img_msg));
  return {{time_of_validity, image, img_msg.camera_serial_number}};
}

}  // namespace filtering
}  // namespace jet
