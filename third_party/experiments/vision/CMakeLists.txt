if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "DEBUG")
  # SET(CMAKE_BUILD_TYPE "RELEASE")
  # SET(CMAKE_BUILD_TYPE "RELWITHDEBINFO")
  # SET(CMAKE_BUILD_TYPE "MINSIZEREL")
endif()

add_library(features features.cc)
target_link_libraries(features ${OpenCV_LIBS})
add_test(features_test features_test.cc features ${OpenCV_LIBS})

add_test(camera_model_test camera_model_test.cc ${OpenCV_LIBS})
add_test(robust_estimator_test robust_estimator_test.cc "")

add_library(visualization visualization.cc)
target_link_libraries(visualization ${OpenCV_LIBS})

add_library(io io.cc)
target_link_libraries(io ${OpenCV_LIBS})

add_library(histogram_equalize histogram_equalize.cc)
target_link_libraries(histogram_equalize features ${OpenCV_LIBS})
add_test(histogram_equalize_test histogram_equalize_test.cc histogram_equalize ${OpenCV_LIBS})

add_library(image_align image_align.cc)
target_link_libraries(image_align ${OpenCV_LIBS})
add_test(image_align_test image_align_test.cc image_align ${OpenCV_LIBS})

add_library(images images.cc)

add_executable(run_map run_map.cc)
target_link_libraries(run_map features images histogram_equalize io ${OpenCV_LIBS})

# find_package(realsense)
# add_executable(real_sense_demo real_sense_demo.cc)
# target_link_libraries(real_sense_demo features images histogram_equalize realsense /usr/local/lib/x86_64-linux-gnu/librealsense.so.1 ${OpenCV_LIBS})

add_executable(calibration_sim calibration_sim.cc)
target_link_libraries(
    calibration_sim
    image_align
    features
    images
    histogram_equalize
    io

    window_3d
    window_manager
    image
    frame
    image_frame
    simple_geometry

    ray

    ${OpenCV_LIBS})



# add_executable(calibrate calibrate.cc)
# target_link_libraries(
#     calibrate
#     image_align
#     features
#     images
#     histogram_equalize
#     io
#
#     window_3d
#     window_manager
#     image
#     frame
#     image_frame
#     simple_geometry
#
#
#     ${OpenCV_LIBS}
# )

add_executable(quick_experiment quick_experiment.cc)
target_link_libraries(quick_experiment features images histogram_equalize io ${OpenCV_LIBS})

add_subdirectory(bootstrapping)
add_subdirectory(render)
