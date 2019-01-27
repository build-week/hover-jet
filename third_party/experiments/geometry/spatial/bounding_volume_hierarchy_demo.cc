#include "viewer/colors/viridis.hh"
#include "viewer/primitives/box.hh"
#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

#include "geometry/import/read_stl.hh"
#include "geometry/spatial/bounding_volume_hierarchy.hh"
#include "geometry/spatial/sphere_volume.hh"
#include "geometry/spatial/triangle_volume.hh"

#include "eigen_helpers.hh"
#include "util/heap.hh"

#include <map>
#include <set>
#include <stack>

using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;

void verify_all_leaves_unique(const geometry::spatial::BoundingVolumeHierarchy &bvh) {
  std::set<int> already_used;
  for (const auto &node : bvh.tree()) {
    if (node.is_leaf) {
      for (int k = node.leaf.start; k < node.leaf.end; ++k) {
        // EXPECT_EQ(already_used.count(k), 0u);
        already_used.insert(k);
      }
    }
  }
}

void draw_children(const geometry::spatial::BoundingVolumeHierarchy &bvh, int base_node_ind) {
  auto win = viewer::get_window3d("Window A");
  auto draw_children_geometry = win->add_primitive<viewer::SimpleGeometry>();

  Heap<int> stack;
  stack.push(base_node_ind);
  while (!stack.empty()) {
    const int next_index = stack.pop();
    const auto &node = bvh.tree()[next_index];
    if (node.is_leaf) {
      for (int k = node.leaf.start; k < node.leaf.end; ++k) {
        const auto &aabb = bvh.aabb()[k];
        viewer::AxisAlignedBox gl_aabb;
        gl_aabb.lower = aabb.bbox.lower();
        gl_aabb.upper = aabb.bbox.upper();

        draw_children_geometry->add_box(gl_aabb);
      }
    } else {
      stack.push(node.node.left_child_index);
      stack.push(node.node.left_child_index + 1);
    }
  }
  draw_children_geometry->flush();
  win->spin_until_step();
  draw_children_geometry->clear();
}

void climb_tree(const geometry::spatial::BoundingVolumeHierarchy &bvh) {
  auto win = viewer::get_window3d("Window A");
  auto tree_climb_geometry = win->add_primitive<viewer::SimpleGeometry>();

  // Explore first branch every time
  struct ExplorationCandidate {
    int index;
    int depth;
  };
  ExplorationCandidate cnd;
  cnd.index = 0;
  cnd.depth = 0;

  Heap<ExplorationCandidate> heap(
      [](const ExplorationCandidate &a, const ExplorationCandidate &b) { return a.depth < b.depth; });
  heap.push(cnd);

  std::map<int, Eigen::Vector4d> colors;
  const auto tree = bvh.tree();
  while (heap.size()) {
    const auto next = heap.pop();
    const auto &node = tree[next.index];

    if (!node.is_leaf) {
      heap.push({node.node.left_child_index, next.depth + 1});
      heap.push({node.node.left_child_index + 1, next.depth + 1});

      auto bbox = tree[node.node.left_child_index].bounding_box;
      bbox.expand(tree[node.node.left_child_index + 1].bounding_box);

      // if (next.depth != 3) {
      // continue;
      // }

      viewer::AxisAlignedBox aabb;
      aabb.lower = node.bounding_box.lower();
      aabb.upper = node.bounding_box.upper();

      if (colors.count(next.depth) == 0) {
        colors[next.depth] = Eigen::Vector4d::Random();
        const double t = next.depth * (1.0 / 10.0);
        colors[next.depth] = jcc::augment(viewer::colors::viridis(t), 1.0);
      }
      aabb.color = colors[next.depth];

      tree_climb_geometry->add_box(aabb);
      draw_children(bvh, next.index);


      win->spin_until_step();
    }
  }
}

void demo_intersection() {
  auto win = viewer::get_window3d("Window A");

  const std::string file_path = "/home/jacob/repos/experiments/data/test_stuff2.stl";
  const auto tri = *geometry::import::read_stl(file_path);
  auto scene_geometry = win->add_primitive<viewer::SimpleGeometry>();
  auto visitor_geometry = win->add_primitive<viewer::SimpleGeometry>();

  std::vector<geometry::spatial::Volume *> tri_ptrs;
  tri_ptrs.reserve(tri.triangles.size());
  std::vector<geometry::spatial::TriangleVolume> triangles;
  triangles.reserve(tri.triangles.size());

  for (size_t k = 0; k < tri.triangles.size(); ++k) {
    triangles.emplace_back(tri.triangles[k].vertices);
    tri_ptrs.push_back(&triangles.back());
  }

  for (size_t k = 0; k < tri.triangles.size(); ++k) {
    scene_geometry->add_line({tri.triangles[k].vertices[0], tri.triangles[k].vertices[1], Vec4(0.8, 0.8, 0.8, 0.4)});
    scene_geometry->add_line({tri.triangles[k].vertices[1], tri.triangles[k].vertices[2], Vec4(0.8, 0.8, 0.8, 0.4)});
    scene_geometry->add_line({tri.triangles[k].vertices[2], tri.triangles[k].vertices[0], Vec4(0.8, 0.8, 0.8, 0.4)});
  }

  scene_geometry->flush();

  const auto visitor = [&visitor_geometry, &win](const geometry::spatial::BoundingVolumeHierarchy::TreeElement &element,
                                                 const bool intersected) {
    viewer::AxisAlignedBox aabb;
    aabb.lower = element.bounding_box.lower();
    aabb.upper = element.bounding_box.upper();
    if (element.is_leaf) {
      aabb.color = Eigen::Vector4d(0.0, 0.0, 1.0, 0.8);
    } else {
      aabb.color = Eigen::Vector4d(1.0, 0.0, 0.0, 1.0);
    }

    if (intersected) {
      aabb.color(1) = 1.0;
    }

    visitor_geometry->add_box(aabb);
    visitor_geometry->flush();
    win->spin_until_step();
  };
  geometry::spatial::BoundingVolumeHierarchy bvh;
  bvh.build(tri_ptrs);

  {
    geometry::Ray ray;
    ray.origin = Vec3(0.0, 0.0, 0.0);
    ray.direction = Vec3(1.0, 1.0, 1.0).normalized();

    scene_geometry->add_ray(ray, 10.0, Vec4(1.0, 0.0, 0.0, 1.0));
    const auto intersection = bvh.intersect(ray, visitor);
    // EXPECT_FALSE(intersection.intersected);
  }
  {
    visitor_geometry->clear();
    geometry::Ray ray;
    ray.origin = Vec3(0.0, 0.0, 0.75);
    ray.direction = Vec3(-1.0, -1.0, 0.0).normalized();

    scene_geometry->add_ray(ray, 10.0, Vec4(1.0, 0.0, 0.0, 1.0));
    const auto intersection = bvh.intersect(ray, visitor);
    // EXPECT_TRUE(intersection.intersected);
    constexpr double EPS = 1e-4;
    // EXPECT_NEAR(intersection.distance, 2.80121, EPS);
  }

  visitor_geometry->flush();
  win->spin_until_step();
  visitor_geometry->clear();
}

void demo_bounding_volumes() {
  auto win = viewer::get_window3d("Window A");

  const std::string file_path = "/home/jacob/repos/experiments/data/test_stuff2.stl";
  const auto tri = *geometry::import::read_stl(file_path);

  auto scene_geometry = win->add_primitive<viewer::SimpleGeometry>();
  auto visitor_geometry = win->add_primitive<viewer::SimpleGeometry>();

  std::vector<geometry::spatial::Volume *> tri_ptrs;
  tri_ptrs.reserve(tri.triangles.size());
  std::vector<geometry::spatial::TriangleVolume> triangles;
  triangles.reserve(tri.triangles.size());

  for (size_t k = 0; k < tri.triangles.size(); ++k) {
    triangles.emplace_back(tri.triangles[k].vertices);
    tri_ptrs.push_back(&triangles.back());
  }

  win->spin_until_step();

  {
    geometry::spatial::BoundingVolumeHierarchy bvh;
    bvh.build(tri_ptrs);
    verify_all_leaves_unique(bvh);
    climb_tree(bvh);
  }
  return;

  std::map<int, Eigen::Vector4d> colors;
  for (int stop_depth = 0; stop_depth < 10; ++stop_depth) {
    const auto visitor = [&visitor_geometry, &win, &colors, stop_depth](const geometry::spatial::BoundingBox<3> &box,
                                                                        int depth, bool leaf) {
      if ((depth != stop_depth) && !leaf) {
        return;
      }
      viewer::AxisAlignedBox aabb;
      aabb.lower = box.lower();
      aabb.upper = box.upper();

      if (colors.count(depth) == 0) {
        colors[depth] = Eigen::Vector4d::Random();
      }
      if (leaf) {
        aabb.color = Eigen::Vector4d(0.0, 1.0, 0.0, 0.8);
      } else {
        aabb.color = colors[depth];
      }
      aabb.color[3] = 0.6;
      aabb.color[0] = 1.0;

      visitor_geometry->add_box(aabb);
    };
    geometry::spatial::BoundingVolumeHierarchy bvh;
    bvh.build(tri_ptrs, visitor);
    visitor_geometry->flush();
    win->spin_until_step();
    visitor_geometry->clear();
  }
  win->spin_until_step();
}

int main() {
  demo_intersection();
  demo_bounding_volumes();
}