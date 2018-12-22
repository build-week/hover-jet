add_library(rpi_bno055 RPi_BNO055.cpp)
target_link_libraries(rpi_bno055 i2c)

add_executable(example Example_Data_Raw.cpp)
target_link_libraries(example rpi_bno055)