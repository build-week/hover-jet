//%deps(yaml-cpp)
#include <yaml-cpp/yaml.h>
#include <vector>
#include "third_party/experiments/sophus.hh"
#include "third_party/experiments/eigen.hh"


namespace jet {


struct FiducialDescription
{
    SE3 tag_from_world;
    int tag_size_squares;
    double arcode_width_mm;
    double arcode_gap_mm;
};

struct CameraExtrinsics
{
    SE3 camera_from_frame;
};

FiducialDescription get_fiducial_pose();
CameraExtrinsics get_camera_extrinsics();

}  //  namespace jet

