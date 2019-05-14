#include <chrono>
#include <iostream>
#include <thread>

#include "embedded/imu_driver/imu_driver.hh"

#include "third_party/experiments/estimation/time_point.hh"

namespace jet {
namespace embedded {

void go(const uint8_t imu_id) {
  ImuDriver driver;
  const bool initialized_successfully = driver.initialize("/dev/i2c-1", imu_id);
  assert(initialized_successfully);

  while (true) {
    const auto t1 = jcc::now();
    const Eigen::Vector3d drv = driver.read_accel_mpss().transpose();
    if (drv == Eigen::Vector3d::Zero()) {
      std::cout << "(1) All zeros, this is the problem, @Mason" << std::endl;
      // Reinitializing seems to fix it, uncomment the next line if you want to try that
      // driver.initialize("/dev/i2c-1", imu_id);
    }

    std::cout << "\tAccel(1): " << driver.read_accel_mpss().transpose() << "\n";
    std::cout << "\tGyro(1) : " << driver.read_gyro_radps().transpose() << "\n";
    std::cout << "\tMag(1)  : " << driver.read_magnetometer_utesla().transpose() << std::endl;
    const auto t2 = jcc::now();
    std::cout << "(Sampling time: " << estimation::to_seconds(t2 - t1) << ") -------" << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(driver.sample_period_ms()));
  }
}

}  // namespace embedded
}  // namespace jet

// Pass *any* argument, and this will use imu 2
//
int main(int argc, const char** argv) {
  const bool use_imu_2 = argc > 1;

  constexpr uint8_t IMU_1_ID = 0x28;
  constexpr uint8_t IMU_2_ID = 0x29;

  const uint8_t imu_id = use_imu_2 ? IMU_2_ID : IMU_1_ID;

  const std::string report_string = use_imu_2 ? "Opening IMU 2" : "Opening IMU 1";

  std::cout << report_string << std::endl;

  jet::embedded::go(imu_id);
}
