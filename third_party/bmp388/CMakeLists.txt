add_library(bosch_bmp bmp3.c)
target_link_libraries(bosch_bmp i2c)

add_library(bmp388 Adafruit_BMP3XX.cc)
target_link_libraries(bmp388 bosch_bmp)

add_executable(bmp388_test bmp388_test.cc)
target_link_libraries(bmp388_test bmp388)