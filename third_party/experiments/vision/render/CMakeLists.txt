add_library(render_from_pose render_from_pose.cc)
target_link_libraries(
    render_from_pose

    window_3d
    window_manager
    image
    frame
    image_frame
    simple_geometry

    ${OpenCV_LIBS}
)

add_test(
    render_from_pose_test
    render_from_pose_test.cc
    render_from_pose

    ${OpenCV_LIBS}
)

add_executable(tracking_simulator tracking_simulator.cc)
target_link_libraries(
    tracking_simulator

    render_from_pose
    features

    window_2d
    window_3d
    window_manager
    plot
    image
    frame
    image_frame
    simple_geometry

    ${OpenCV_LIBS}
)
