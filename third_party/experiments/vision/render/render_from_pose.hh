#pragma once

#include "out.hh"
#include "sophus.hh"

#include "vision/camera_model.hh"

#include "eigen.hh"
#include <opencv2/opencv.hpp>

#include <vector>

namespace slam {
namespace render {

// Given an image at some SE(3) location in space, render the image a camera would see
//
// @param ref_image The image that is floating in space, which we should attempt to render
// @param cam_model The camera model
// @param image_from_camera A member of SE(3) taking the camera from to the image frame
// @returns The rendered image
cv::Mat render_from_pose(const cv::Mat& ref_image, const CameraModel& cam_model, const SE3& image_from_camera);
}
}