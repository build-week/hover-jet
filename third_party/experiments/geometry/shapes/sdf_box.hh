#pragma once

#include "geometry/spatial/bounding_box.hh"
#include "geometry/tri_mesh.hh"

#include "eigen.hh"
#include "out.hh"

namespace geometry {
namespace shapes {

// A contraption for managing a *sampled* sdf
//
// TODO:
// - Tri-linear interpolation for neighboring voxels
// - Exact distance at the box surface
//
class SampledSdf {
 public:
  SampledSdf(const TriMesh& mesh, double voxel_size_m = 0.1);

  // Computes an estimate of the distance between `p` and the contained mesh
  //
  // Method:
  //   If p is within the bounding box, return the signed distance at the nearest voxel
  //
  //   If p is *outside* of the box:
  //
  // 1. Find the nearest point on the box
  // 2. Find [roughly] the distance from that point on the box to the mesh
  // 3. Estimate dist(p, mesh) as dist(p, box) + dist(q, p)
  //    where q is the nearest point on the box to p
  //
  double signed_distance(const Eigen::Vector3d& p) const;

  // Compute the voxel index corresponding to some point p
  int voxel_index_for_position(const Eigen::Vector3d& p) const;

  // Compute the voxel center corresponding to a voxel index
  Eigen::Vector3d position_for_voxel_index(int i) const;

  //
  //////////////////////////////////////////////////
  //

  // Get the bounding box for the sampled sdf
  const spatial::BoundingBox<3>& bounding_box() const {
    return bbox_;
  }

  // Get the list of voxel signed distances
  const std::vector<double>& get_signed_distances() const {
    return signed_distances_;
  }

  // Get the list of nearest triangles to voxel centers
  //
  // ```
  // Vec3 p = ...;
  // int i = sdf.voxel_index_for_position(p);
  // auto nearest_tri = triangles[indices()[i]];
  // ```
  //
  const std::vector<int>& indices() const {
    return indices_;
  }

 private:
  void populate_voxel_grid(const TriMesh& mesh,
                           Out<std::vector<double>> distances,
                           Out<std::vector<int>> indices) const;

  // The bounding box for this voxel grid
  spatial::BoundingBox<3> bbox_;

  // Signed distance to the mesh [cached]
  std::vector<double> signed_distances_;

  // Indices of the nearest triangle to each voxel
  std::vector<int> indices_;

  Eigen::Matrix<int, 3, 1> voxels_per_axis_;
  double voxel_size_m_ = -1.0;
};

}  // namespace shapes
}  // namespace geometry