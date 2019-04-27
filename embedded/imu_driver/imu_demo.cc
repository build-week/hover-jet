#include <chrono>
#include <iostream>
#include <thread>

#include "embedded/imu_driver/imu_driver.hh"

#include "third_party/experiments/estimation/time_point.hh"

namespace jet {
namespace embedded {

void go() {
  ImuDriver driver_1;
  ImuDriver driver_2;
  driver_1.initialize("/dev/i2c-1", 0x28);
  driver_2.initialize("/dev/i2c-1", 0x29);

  while (true) {
    const auto t1 = jcc::now();
    const Eigen::Vector3d accel_mpss_dr1 = driver_1.read_accel_mpss().transpose();
    if (accel_mpss_dr1 == Eigen::Vector3d::Zero()) {
      std::cout << "(1) All zeros, attempting to reinitialize" << std::endl;
      driver_1.initialize("/dev/i2c-1", 0x28);
    }

    std::cout << "\tAccel(1): " << driver_1.read_accel_mpss().transpose() << "\n";
    std::cout << "\tGyro(1) : " << driver_1.read_gyro_radps().transpose() << "\n";
    std::cout << "\tMag(1)  : " << driver_1.read_magnetometer_utesla().transpose() << std::endl;
    const auto t2 = jcc::now();
    std::cout << "-------" << std::endl;

    const Eigen::Vector3d accel_mpss_dr2 = driver_2.read_accel_mpss().transpose();
    if (accel_mpss_dr2 == Eigen::Vector3d::Zero()) {
      std::cout << "(2) All zeros, attempting to reinitialize" << std::endl;
      driver_2.initialize("/dev/i2c-1", 0x29);
    }
    std::cout << "\tAccel(2): " << accel_mpss_dr2 << "\n";
    std::cout << "\tGyro(2) : " << driver_2.read_gyro_radps().transpose() << "\n";
    std::cout << "\tMag(2)  : " << driver_2.read_magnetometer_utesla().transpose() << std::endl;
    const auto t3 = jcc::now();

    std::cout << "\n\n" << std::endl;
    std::cout << "IMU 1: " << estimation::to_seconds(t2 - t1) << "s" << std::endl;
    std::cout << "IMU 2: " << estimation::to_seconds(t3 - t2) << "s" << std::endl;
    std::cout << "Total: " << estimation::to_seconds(t3 - t1) << "s" << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(driver_1.sample_period_ms()));
  }
}

}  // namespace embedded
}  // namespace jet

int main() {
  jet::embedded::go();
}
