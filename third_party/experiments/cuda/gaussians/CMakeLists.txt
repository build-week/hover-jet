cmake_minimum_required(VERSION 2.8)
find_package(CUDA QUIET REQUIRED)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )

#########################
# Eigen
#

set(BASEPATH "${CMAKE_CURRENT_SOURCE_DIR}/..")
INCLUDE_DIRECTORIES("${BASEPATH}")
INCLUDE_DIRECTORIES("${BASEPATH}/common/inc")

include_directories("${BASEPATH}/../third_party/Sophus")
include_directories("${BASEPATH}/../third_party/eigen3")

set (CMAKE_CXX_FLAGS "--std=c++11 ${CMAKE_CXX_FLAGS}")

list(APPEND CUDA_NVCC_FLAGS "-std=c++11;-O2;-DVERBOSE;-gencode;arch=compute_61,code=sm_61;--expt-relaxed-constexpr")

SET(CUDA_PROPAGATE_HOST_FLAGS OFF)

# Specify binary name and source file to build it from
cuda_add_executable(
    gaussians
    gaussians.cc
    cugaussians.cu
)

target_link_libraries(gaussians ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})
