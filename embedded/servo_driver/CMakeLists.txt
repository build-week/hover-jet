cmake_minimum_required(VERSION 2.8)
project(servo_test)

# set up our paths
set(SOURCE_DIR .)
set(LIB_I2C ../../third-party)

# we want to build the driver
include_directories(${SOURCE_DIR})
include_directories(${LIB_I2C})

# the driver is a library
add_library(servo_driver SHARED servo_driver.cc)
add_library(i2c SHARED ${LIB_I2C}/i2c.c)

# we need the i2c lib
#find_package(libi2c COMPONENTS REQUIRED)
#target_link_libraries(servo_test libi2c)

# make our executable, and link libi2c
add_executable(servo_test servo_test.cc)
target_link_libraries(servo_test i2c servo_driver)
target_link_libraries(servo_driver i2c)

#install(TARGETS i2c DESTINATION bin)
#install(FILES i2c.h DESTINATION include)