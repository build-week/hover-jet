cmake_minimum_required(VERSION 2.8)
project(pwm_test)

include_directories("${CMAKE_CURRENT_SOURCE_DIR}")

# make our executable, and link libi2c
add_executable(servo_test ${SERVO_DRIVER}/servo_test.cc)
target_link_libraries(servo_test i2c servo_driver)
target_link_libraries(servo_driver i2c)

add_subdirectory(embedded)
add_subdirectory(third-party)

