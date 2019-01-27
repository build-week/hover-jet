#include "geometry/shapes/sdf_box.hh"

#include "geometry/spatial/fit_bounding_box.hh"
#include "geometry/spatial/nearest_triangle.hh"

namespace geometry {
namespace shapes {

using Vec3 = Eigen::Vector3d;

Vec3 SampledSdf::position_for_voxel_index(int i) const {
  const int& nx = voxels_per_axis_[0];
  const int& ny = voxels_per_axis_[1];

  const Vec3 p(i % nx, (i / nx) % ny, i / (nx * ny));

  // Voxel center coordinates
  const Vec3 offset =
      bbox_.lower() + Vec3(voxel_size_m_, voxel_size_m_, voxel_size_m_) * 0.5;
  return (p * voxel_size_m_) + offset;
}

int SampledSdf::voxel_index_for_position(const Vec3& p) const {
  const Vec3 offset =
      bbox_.lower() + Vec3(voxel_size_m_, voxel_size_m_, voxel_size_m_) * 0.5;

  // Normalized coordinates. Center of the bottom-corner voxel is the origin
  const Vec3 nrm_pt = (p - offset) / voxel_size_m_;
  const Eigen::Vector3i nrm_pt_i = nrm_pt.cast<int>();

  const int& nx = voxels_per_axis_[0];
  const int& ny = voxels_per_axis_[1];

  std::cout << "::" << nrm_pt.transpose() << std::endl;
  const int i = nx * ((ny * nrm_pt_i.z()) + nrm_pt_i.y()) + nrm_pt_i.x();
  return i;
}

void SampledSdf::populate_voxel_grid(const TriMesh& mesh,
                                     Out<std::vector<double>> distances,
                                     Out<std::vector<int>> indices) const {
  distances->clear();
  indices->clear();
  // No more than 1 million voxels (4mb)
  assert(voxel_size_m_ > 0.0);
  constexpr int MAX_SIZE = 1e6;

  const int total_voxels =
      voxels_per_axis_[0] * voxels_per_axis_[1] * voxels_per_axis_[2];
  assert(total_voxels < MAX_SIZE);

  distances->resize(total_voxels);
  indices->resize(total_voxels);

  for (int i = 0; i < total_voxels; ++i) {
    const Vec3 voxel_center = position_for_voxel_index(i);
    const auto tri = spatial::find_nearest_triangle(voxel_center, mesh);
    (*distances)[i] = tri.distance;
    (*indices)[i] = tri.simplex_index;
  }
}

SampledSdf::SampledSdf(const TriMesh& mesh, double meters_per_voxel) {
  voxel_size_m_ = meters_per_voxel;
  {
    bbox_ = spatial::fit_bounding_box(mesh);
    const Vec3 bbox_size_m = bbox_.upper() - bbox_.lower();
    const Vec3 num_voxels_per_axis = bbox_size_m / meters_per_voxel;
    voxels_per_axis_ = num_voxels_per_axis.array().ceil().cast<int>();
  }

  populate_voxel_grid(mesh, out(signed_distances_), out(indices_));
}

// TODO: Remove the .at's once this code is verified
double SampledSdf::signed_distance(const Vec3& p) const {
  if (bbox_.contains(p)) {
    const int voxel_index = voxel_index_for_position(p);
    return signed_distances_.at(voxel_index);
  } else {
    // Nearest point on box
    const Vec3 nearest_on_bbox = bbox_.nearest_point(p);
    const Vec3 toward_box_center = (bbox_.center() - nearest_on_bbox).normalized();
    const Vec3 inside_box = nearest_on_bbox + (toward_box_center * voxel_size_m_ * 0.5);

    // Approximate distance from box surface to mesh
    const int nearest_voxel_ind = voxel_index_for_position(inside_box);
    const Vec3 nearest_voxel_center = position_for_voxel_index(nearest_voxel_ind);
    const double approx_dist_to_mesh =
        (p - nearest_voxel_center).norm() + signed_distances_.at(nearest_voxel_ind);

    // Distance to the box + distance to the mesh at the surface of the box
    return bbox_.ud_box(p) + approx_dist_to_mesh;
  }
}

}  // namespace shapes
}  // namespace geometry