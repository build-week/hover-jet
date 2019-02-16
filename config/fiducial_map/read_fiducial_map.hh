//%deps(yaml-cpp)
#include <yaml-cpp/yaml.h>
#include <vector>
#include "third_party/experiments/sophus.hh"
#include "third_party/experiments/eigen.hh"


namespace jet {


struct fiducial_pose
{
    SE3 tag_from_world;
    int tag_size_squares;
    double arcode_width_mm;
    double arcode_gap_mm;
};

struct camera_extrinsics_struct
{
    SE3 camera_from_frame;
};

fiducial_pose get_fiducial_pose();
camera_extrinsics_struct get_camera_extrinsics();

}  //  namespace jet

