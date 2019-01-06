#include <chrono>
#include <iostream>
#include <thread>

#include "embedded/imu_driver/imu_driver.hh"

namespace jet {
namespace embedded {

void go() {
  ImuDriver driver;
  driver.initialize();

  while (true) {
    std::cout << driver.read_accel_mpss().transpose() << "; ";
    std::cout << driver.read_gyro_radps().transpose() << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(driver.sample_period_ms()));
  }
}

}  // namespace embedded
}  // namespace jet

int main() {
  jet::embedded::go();
}

