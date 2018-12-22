cmake_minimum_required(VERSION 2.8)
project(pwm_test)

# set up our paths
set(PWM_DRIVER embedded/pwm_driver/)
set(SERVO_DRIVER embedded/servo_driver/)
set(LIB_I2C third-party/i2c/)

# included dirs
include_directories("${CMAKE_CURRENT_SOURCE_DIR}")
include_directories(${LIB_I2C})
include_directories(${PWM_DRIVER})
include_directories(${SERVO_DRIVER})

# the driver is a library
add_library(pwm_driver SHARED ${PWM_DRIVER}/pwm_driver.cc)
add_library(servo_driver SHARED ${SERVO_DRIVER}/servo_driver.cc)
add_library(i2c SHARED ${LIB_I2C}/i2c.c)

# make our executable, and link libi2c
add_executable(pwm_test ${PWM_DRIVER}/pwm_test.cc)
target_link_libraries(pwm_test i2c pwm_driver)
target_link_libraries(pwm_driver i2c)

add_executable(servo_test ${SERVO_DRIVER}/servo_test.cc)
target_link_libraries(servo_test servo_driver)
target_link_libraries(servo_driver pwm_driver i2c)
