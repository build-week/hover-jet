
list(APPEND CUDA_NVCC_FLAGS "-std=c++11;-O2;-DVERBOSE;-gencode;arch=compute_61,code=sm_61;--expt-relaxed-constexpr")

SET(CUDA_PROPAGATE_HOST_FLAGS OFF)

# Specify binary name and source file to build it from
# cuda_add_executable(
#     gaussians
#     gaussians.cc
#     cugaussians.cu
# )



# add_library(window_cuda window_cuda.cc)
# target_link_libraries(
#     window_cuda

#     projection
#     window_manager

#     ${OPENGL_LIBRARIES}
#     ${GLFW_LIBRARIES}
# )
