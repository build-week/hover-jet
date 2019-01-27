#include "viewer/primitives/box.hh"

#include <GL/glew.h>

namespace viewer {

void Box::draw() const {
  glBegin(GL_QUADS);
  // top
  glColor3f(1.0f, 0.0f, 0.0f);
  glNormal3f(0.0f, 1.0f, 0.0f);
  glVertex3f(-0.5f, 0.5f, 0.5f);
  glVertex3f(0.5f, 0.5f, 0.5f);
  glVertex3f(0.5f, 0.5f, -0.5f);
  glVertex3f(-0.5f, 0.5f, -0.5f);

  glEnd();

  glBegin(GL_QUADS);
  // front
  glColor3f(0.0f, 1.0f, 0.0f);
  glNormal3f(0.0f, 0.0f, 1.0f);
  glVertex3f(0.5f, -0.5f, 0.5f);
  glVertex3f(0.5f, 0.5f, 0.5f);
  glVertex3f(-0.5f, 0.5f, 0.5f);
  glVertex3f(-0.5f, -0.5f, 0.5f);

  glEnd();

  glBegin(GL_QUADS);
  // right
  glColor3f(0.0f, 0.0f, 1.0f);
  glNormal3f(1.0f, 0.0f, 0.0f);
  glVertex3f(0.5f, 0.5f, -0.5f);
  glVertex3f(0.5f, 0.5f, 0.5f);
  glVertex3f(0.5f, -0.5f, 0.5f);
  glVertex3f(0.5f, -0.5f, -0.5f);

  glEnd();

  glBegin(GL_QUADS);
  // left
  glColor3f(0.0f, 0.0f, 0.5f);
  glNormal3f(-1.0f, 0.0f, 0.0f);
  glVertex3f(-0.5f, -0.5f, 0.5f);
  glVertex3f(-0.5f, 0.5f, 0.5f);
  glVertex3f(-0.5f, 0.5f, -0.5f);
  glVertex3f(-0.5f, -0.5f, -0.5f);

  glEnd();

  glBegin(GL_QUADS);
  // bottom
  glColor3f(0.5f, 0.0f, 0.0f);
  glNormal3f(0.0f, -1.0f, 0.0f);
  glVertex3f(0.5f, -0.5f, 0.5f);
  glVertex3f(-0.5f, -0.5f, 0.5f);
  glVertex3f(-0.5f, -0.5f, -0.5f);
  glVertex3f(0.5f, -0.5f, -0.5f);

  glEnd();

  glBegin(GL_QUADS);
  // back
  glColor3f(0.0f, 0.5f, 0.0f);
  glNormal3f(0.0f, 0.0f, -1.0f);
  glVertex3f(0.5f, 0.5f, -0.5f);
  glVertex3f(0.5f, -0.5f, -0.5f);
  glVertex3f(-0.5f, -0.5f, -0.5f);
  glVertex3f(-0.5f, 0.5f, -0.5f);

  glEnd();
}
}
