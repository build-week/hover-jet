#include <GL/freeglut.h>

#include <cuda_gl_interop.h>
#include <cuda_profiler_api.h>
#include <cuda_runtime.h>
#include <driver_functions.h>
#include <vector_functions.h>
#include <vector_types.h>

#include <cstdio>
#include <cstdlib>

#include <helper_cuda.h>
#include <helper_math.h>

#include <cuda_gl_interop.h>
#include <helper_gl.h>

extern "C" void render_kernel(dim3 gridSize, dim3 blockSize, uint *d_output,
                              uint imageW, uint imageH, float moat,
                              float2 hello);

int iDivUp(int a, int b) { return (a % b != 0) ? (a / b + 1) : (a / b); }

struct State {
  float2 view_center = make_float2(0.0, 0.0);
  float2 prev_view_center = make_float2(0.0, 0.0);
  float2 prev_mouse_pos = make_float2(0.0, 0.0);

  // OpenGL pixel buffer object
  GLuint pbo = 0;

  // OpenGL texture object
  GLuint tex = 0;

  // CUDA Graphics Resource (to transfer PBO)
  struct cudaGraphicsResource *cuda_pbo_resource;

  // Image render dimensions
  uint width = 512;
  uint height = 512;

  float scale = 1.0;

  dim3 blockSize = dim3(16, 16);
  dim3 gridSize = dim3(iDivUp(width, blockSize.x), iDivUp(height, blockSize.y));

} gstate;

void initGL(int *argc, char **argv) {
  // initialize GLUT callback functions
  glutInit(argc, argv);
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
  glutInitWindowSize(gstate.width, gstate.height);
  glutCreateWindow("CUDA volume rendering");

  if (!isGLVersionSupported(2, 0) ||
      !areGLExtensionsSupported("GL_ARB_pixel_buffer_object")) {
    printf("Required OpenGL extensions are missing.");
    exit(EXIT_SUCCESS);
  }
}

void initPixelBuffer() {
  if (gstate.pbo) {
    // unregister this buffer object from CUDA C
    checkCudaErrors(cudaGraphicsUnregisterResource(gstate.cuda_pbo_resource));

    // delete old buffer
    glDeleteBuffers(1, &(gstate.pbo));
    glDeleteTextures(1, &(gstate.tex));
  }

  // create pixel buffer object for display
  glGenBuffers(1, &(gstate.pbo));
  glBindBuffer(GL_PIXEL_UNPACK_BUFFER_ARB, gstate.pbo);
  glBufferData(GL_PIXEL_UNPACK_BUFFER_ARB,
               gstate.width * gstate.height * sizeof(GLubyte) * 4, 0,
               GL_STREAM_DRAW_ARB);
  glBindBuffer(GL_PIXEL_UNPACK_BUFFER_ARB, 0);

  // register this buffer object with CUDA
  checkCudaErrors(
      cudaGraphicsGLRegisterBuffer(&(gstate.cuda_pbo_resource), gstate.pbo,
                                   cudaGraphicsMapFlagsWriteDiscard));

  // create texture for display
  glGenTextures(1, &(gstate.tex));
  glBindTexture(GL_TEXTURE_2D, gstate.tex);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, gstate.width, gstate.height, 0,
               GL_RGBA, GL_UNSIGNED_BYTE, NULL);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glBindTexture(GL_TEXTURE_2D, 0);
}

void render() {
  uint *d_output;
  checkCudaErrors(cudaGraphicsMapResources(1, &(gstate.cuda_pbo_resource), 0));

  size_t num_bytes;
  checkCudaErrors(cudaGraphicsResourceGetMappedPointer(
      (void **)&d_output, &num_bytes, gstate.cuda_pbo_resource));

  checkCudaErrors(cudaMemset(d_output, 0, gstate.width * gstate.height * 4));

  // Create performance metrics
  cudaEvent_t start, stop;
  cudaEventCreate(&start);
  cudaEventCreate(&stop);

  cudaEventRecord(start);

  // Render the acutal image
  render_kernel(gstate.gridSize, gstate.blockSize, d_output, gstate.width,
                gstate.height, gstate.scale, gstate.view_center);

  cudaEventRecord(stop);
  cudaEventSynchronize(stop);

  float milliseconds = 0;
  cudaEventElapsedTime(&milliseconds, start, stop);

  // std::cout << "Rendering took " << milliseconds << " ms" << std::endl;
  checkCudaErrors(
      cudaGraphicsUnmapResources(1, &(gstate.cuda_pbo_resource), 0));
}

// display results using OpenGL (called by GLUT)
void display() {
  // use OpenGL to build view matrix
  GLfloat modelView[16];
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  glGetFloatv(GL_MODELVIEW_MATRIX, modelView);
  glPopMatrix();
  render();

  // Display results
  glClear(GL_COLOR_BUFFER_BIT);
  glDisable(GL_DEPTH_TEST);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

  //
  // Prepare render to texture
  //
  glBindBuffer(GL_PIXEL_UNPACK_BUFFER_ARB, gstate.pbo);
  glBindTexture(GL_TEXTURE_2D, gstate.tex);
  glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, gstate.width, gstate.height, GL_RGBA,
                  GL_UNSIGNED_BYTE, 0);
  glBindBuffer(GL_PIXEL_UNPACK_BUFFER_ARB, 0);

  //
  // Draw the texture on the simplest quad
  //
  // Draw textured quad
  glEnable(GL_TEXTURE_2D);
  glBegin(GL_QUADS);

  // This is just a quad that occupies the whole screen
  glTexCoord2f(0, 0);
  glVertex2f(-1, -1);
  glTexCoord2f(1, 0);
  glVertex2f(1, -1);
  glTexCoord2f(1, 1);
  glVertex2f(1, 1);
  glTexCoord2f(0, 1);
  glVertex2f(-1, 1);

  glEnd();

  glDisable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, 0);

  //
  // And it's over
  //

  glutSwapBuffers();
  glutReportErrors();
}

void idle() { glutPostRedisplay(); }

void motion(int x, int y) {
  float2 mouse_pos = make_float2(x, y);

  float2 delta_pos =
      (mouse_pos - gstate.prev_mouse_pos) / (gstate.height / 2.0);

  gstate.view_center =
      make_float2(-delta_pos.x, delta_pos.y) + gstate.prev_view_center;

  glutPostRedisplay();
}

void mouse(int button, int state, int x, int y) {
  int mods;

  float2 mouse_pos = make_float2(x, y);
  if (state == GLUT_DOWN) {
    gstate.prev_mouse_pos = mouse_pos;
  } else if (state == GLUT_UP) {
    gstate.prev_view_center = gstate.view_center;
  }

  if ((button == 3) || (button == 4))  // It's a wheel event
  {
    if (state == GLUT_UP) return;

    gstate.scale *= (button == 3) ? 0.9 : 1.1;

    // printf("Scroll %s At %d %d\n", (button == 3) ? "Up" : "Down", x, y);
  }

  std::cout << gstate.view_center.x << " , " << gstate.view_center.y
            << std::endl;
}

void keyboard(unsigned char key, int x, int y) {
  switch (key) {
    case 27:
      glutDestroyWindow(glutGetWindow());
      return;

    case 'f':
      break;

    case 'q':
      std::cout << "Attempting to exit" << std::endl;
      glutDestroyWindow(glutGetWindow());
      return;

    default:
      break;
  }

  glutPostRedisplay();
}

int main(int argc, char **argv) {
#if defined(__linux__)
  setenv("DISPLAY", ":0", 0);
#endif
  std::cout << "Jake cuViewer starting\n" << std::endl;

  //
  // Initialize openGL
  //
  initGL(&argc, argv);

  //
  // Initialize glut
  //
  glutMouseFunc(mouse);
  glutMotionFunc(motion);
  glutDisplayFunc(display);
  glutKeyboardFunc(keyboard);
  glutIdleFunc(idle);

  //
  // Zero the pbo
  //
  initPixelBuffer();

  glutMainLoop();
}