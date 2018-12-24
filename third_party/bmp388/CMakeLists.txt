add_library(bosch_bmp bmp3.c)
target_link_libraries(bosch_bmp i2c)

add_library(bmp388 Adafruit_BMP3XX.cc)
target_link_libraries(bmp388 bosch_bmp)