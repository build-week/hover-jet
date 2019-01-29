#include "viewer/primitives/simple_geometry_primitives.hh"

#include <GL/glew.h>
#include "viewer/colors/material.hh"
#include "viewer/gl_aliases.hh"

#include "geometry/perp.hh"

namespace viewer {

using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;

void draw_axes(const Axes &axes) {
  glPushAttrib(GL_CURRENT_BIT | GL_LINE_BIT);
  glLineWidth(1.0);

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glTransform(axes.world_from_axes);
  glScaled(axes.scale, axes.scale, axes.scale);

  if (axes.dotted) {
    glEnable(GL_LINE_STIPPLE);
    glLineStipple(1.0, 0x00FF);
  }
  glLineWidth(axes.line_width);

  glBegin(GL_LINES);

  glColor4d(1.0, 0.0, 0.0, 0.9);
  glVertex(Vec3::UnitX().eval());
  glVertex(Vec3::Zero().eval());

  glColor4d(0.0, 1.0, 0.0, 0.9);
  glVertex(Vec3::UnitY().eval());
  glVertex(Vec3::Zero().eval());

  glColor4d(0.0, 0.0, 1.0, 0.9);
  glVertex(Vec3::UnitZ().eval());
  glVertex(Vec3::Zero().eval());
  glEnd();

  glPopMatrix();
  glPopAttrib();
}

void draw_lines(const std::vector<Line> &lines) {
  glPushAttrib(GL_CURRENT_BIT | GL_LINE_BIT);

  for (const auto &line : lines) {
    glLineWidth(line.width);
    glBegin(GL_LINES);
    glColor(line.color);
    glVertex(line.start);
    glVertex(line.end);
    glEnd();
  }

  glPopAttrib();
}

void draw_polygon(const Polygon &polygon) {
  const int n_points = static_cast<int>(polygon.points.size());
  const Eigen::Vector3d offset(0.0, 0.0, polygon.height);

  glPushAttrib(GL_CURRENT_BIT | GL_LINE_BIT);
  glLineWidth(polygon.width);
  // Draw the height of the poly
  /*
  glBegin(GL_QUADS);
  glColor(polygon.color);
  for (int k = 0; k < n_points; ++k) {
    const auto &start = polygon.points[k];
    const auto &end = polygon.points[(k + 1) % n_points];
    glVertex(start);
    glVertex(end);

    glVertex(Vec3(end + offset));
    glVertex(Vec3(start + offset));
  }
  glEnd();
  */

  /*
  if (polygon.outline) {
    glLineWidth(0.8);
    glBegin(GL_LINE_LOOP);
    glColor(Vec4(Vec4::Ones()));
    for (int k = 0; k < n_points; ++k) {
      const auto &start = polygon.points[k];
      const auto &end = polygon.points[(k + 1) % n_points];
      glVertex(start);
      glVertex(end);

      glVertex(Vec3(end + offset));
      glVertex(Vec3(start + offset));
    }
    glEnd();
  }
  */

  glColor(polygon.color);
  glBegin(GL_TRIANGLE_FAN);
  for (int k = 0; k < n_points; ++k) {
    const auto &point = polygon.points[k];
    glVertex(point);
  }
  glEnd();

  if (polygon.outline) {
    glLineWidth(3.0);
    glColor(Vec4(Vec4::Ones()));
    glBegin(GL_LINE_LOOP);
    for (int k = 0; k < n_points; ++k) {
      const auto &point = polygon.points[k];
      glVertex(point);
    }
    glEnd();
  }

  glPopAttrib();
}

void draw_points(const Points &points) {
  glPushAttrib(GL_POINT_BIT | GL_CURRENT_BIT);
  glColor(points.color);
  glPointSize(points.size);
  glBegin(GL_POINTS);
  for (const auto &pt : points.points) {
    glVertex(pt);
  }
  glEnd();
  glPopAttrib();
}

void draw_colored_points(const ColoredPoints &points) {
  glPushAttrib(GL_POINT_BIT | GL_CURRENT_BIT);
  glPointSize(points.size);
  glBegin(GL_POINTS);
  for (std::size_t k = 0; k < points.points.size(); ++k) {
    glColor(points.colors[k]);
    glVertex(points.points[k]);
  }
  glEnd();
  glPopAttrib();
}

void draw_points2d(const Points2d &points) {
  glPushAttrib(GL_POINT_BIT | GL_CURRENT_BIT);
  glColor(points.color);
  glPointSize(points.size);
  glBegin(GL_POINTS);
  for (const auto &pt : points.points) {
    glVertex3d(pt.x(), pt.y(), points.z_offset);
  }
  glEnd();
  glPopAttrib();
}

void draw_circle(const Vec3 &center,
                 const Vec3 &normal,
                 const double radius,
                 const Vec4 &color) {
  constexpr double CIRCLE_RES = 0.05;

  const Vec3 x_u = geometry::perp(normal);
  const Vec3 y_u = -x_u.cross(normal);

  Eigen::Matrix3d world_from_circle_frame;
  world_from_circle_frame.col(0) = x_u;
  world_from_circle_frame.col(1) = y_u;
  world_from_circle_frame.col(2) = normal;

  glColor(color);
  glBegin(GL_LINE_LOOP);
  for (double t = 0.0; t < (2.0 * M_PI); t += CIRCLE_RES) {
    const Vec3 pt_circle_frm(radius * std::sin(t), radius * std::cos(t), 0.0);
    const Vec3 pt_world_frm = (world_from_circle_frame * pt_circle_frm) + center;
    glVertex(pt_world_frm);
  }
  glEnd();
}

void draw_sphere(const Sphere &sphere) {
  draw_circle(sphere.center, sphere.world_from_sphere * Vec3::UnitX(), sphere.radius,
              sphere.color);
  draw_circle(sphere.center, sphere.world_from_sphere * Vec3::UnitY(), sphere.radius,
              sphere.color);
  draw_circle(sphere.center, sphere.world_from_sphere * Vec3::UnitZ(), sphere.radius,
              sphere.color);
}

void draw_plane_grid(const Plane &plane) {
  const Vec3 &n = plane.plane.u_normal;
  const Vec3 x_dir = geometry::perp(n);
  const Vec3 y_dir = x_dir.cross(n).normalized();

  const int n_lines = 10;
  const double displacement = n_lines * plane.line_spacing;
  const Vec3 offset = plane.plane.u_normal * plane.plane.distance_from_origin;

  const Vec3 y_offset = y_dir * 10.0;
  const Vec3 x_offset = x_dir * 10.0;

  glPushAttrib(GL_CURRENT_BIT);
  glColor(plane.color);
  glBegin(GL_LINES);
  {
    for (double x = -displacement; x <= displacement; x += plane.line_spacing) {
      const Vec3 v = (x * x_dir) + offset;

      // Lines retreating off to infinity by homogeneousness
      // turns out, it doesn't look that good.

      /*glVertex4d(v.x(), v.y(), v.z(), 0.0);
      glVertex4d(y_dir.x(), y_dir.y(), y_dir.z(), 0.0);
      glVertex4d(v.x(), v.y(), v.z(), 1.0);*/

      glVertex(Vec3(v + y_offset));
      glVertex(Vec3(v - y_offset));
    }
    for (double y = -displacement; y <= displacement; y += plane.line_spacing) {
      const Vec3 v = (y * y_dir) + offset;

      glVertex(Vec3(v + x_offset));
      glVertex(Vec3(v - x_offset));
    }
  }
  glEnd();
  glPopAttrib();
}

void draw_point(const Point &point) {
  glPushAttrib(GL_POINT_BIT | GL_CURRENT_BIT);
  glPointSize(point.size);
  glColor(point.color);

  glBegin(GL_POINTS);
  glVertex(point.point);
  glEnd();

  glPopAttrib();
}

void draw_trimesh(const TriMesh &trimesh) {
  glPushAttrib(GL_CURRENT_BIT);

  glPushMatrix();
  glTransform(trimesh.world_from_mesh);

  glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
  if (trimesh.outline) {
    glPushAttrib(GL_LINE_BIT);
    glLineWidth(trimesh.outline_width);
    glBegin(GL_LINES);
    for (const auto &tri : trimesh.mesh.triangles) {
      glVertex(tri.vertices[0]);
      glVertex(tri.vertices[1]);
      glVertex(tri.vertices[2]);
    }
    glEnd();
    glPopAttrib();
  }

  // glEnable(GL_LIGHTING);
  const auto material = colors::get_plastic(trimesh.color);
  colors::gl_material(material);
  glColor(trimesh.color);
  if (trimesh.filled) {
    glBegin(GL_TRIANGLES);
    for (const auto &tri : trimesh.mesh.triangles) {
      glVertex(tri.vertices[0]);
      glVertex(tri.vertices[1]);
      glVertex(tri.vertices[2]);
    }

    glEnd();
  }
  // glDisable(GL_LIGHTING);
  glPopMatrix();

  glPopAttrib();
  // draw the display list
}
}  // namespace viewer
