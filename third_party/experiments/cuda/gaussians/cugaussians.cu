#ifndef _DIST_KERNEL_CU_
#define _DIST_KERNEL_CU_

#include <helper_cuda.h>
#include <helper_math.h>

#define EIGEN_DEFAULT_DENSE_INDEX_TYPE int
#include <Eigen/Dense>

__device__ uint rgbaFloatToInt(float4 rgba) {
  rgba.x = __saturatef(rgba.x);  // clamp to [0.0, 1.0]
  rgba.y = __saturatef(rgba.y);
  rgba.z = __saturatef(rgba.z);
  rgba.w = __saturatef(rgba.w);
  return (uint(rgba.w * 255) << 24) | (uint(rgba.z * 255) << 16) | (uint(rgba.y * 255) << 8) | uint(rgba.x * 255);
}

__global__ void d_render(uint *           d_output,
                         uint             imageW,
                         uint             imageH,
                         float            scale,
                         float2           view_center,
                         Eigen::Vector2f *means,
                         Eigen::Matrix2f *information_matrices,
                         int              N,
                         float            normalization,
                         float            tstep) {
  uint x = blockIdx.x * blockDim.x + threadIdx.x;
  uint y = blockIdx.y * blockDim.y + threadIdx.y;

  if ((x >= imageW) || (y >= imageH)) {
    return;
  }

  // These int-float cast operations aren't cheap
  const float unscaled_u = ((x / (float)imageW) * 2.0f) - 1.0f;
  const float unscaled_v = ((y / (float)imageH) * 2.0f) - 1.0f;

  const float u = scale * (unscaled_u + view_center.x);
  const float v = scale * (unscaled_v + view_center.y);

  Eigen::Vector2f vv    = Eigen::Vector2f(u, v);
  float           value = 0.0f;
  for (int k = 0; k < N; ++k) {
    const Eigen::Vector2f eigv        = (vv - means[k]);
    const float           mahalanobis = (eigv.transpose() * information_matrices[k] * eigv)(0);
    const float           probability = __expf(-0.5f * mahalanobis);
    value += probability;
  }
  const float normalized_value = value * normalization;
  float4      color =
      make_float4(tstep * normalized_value, normalized_value * 0.5f, tstep * normalized_value * normalized_value, 1.0f);

  // This could be optimized
  d_output[y * imageW + x] = rgbaFloatToInt(color);
}

void render_kernel(dim3             gridSize,
                   dim3             blockSize,
                   uint *           d_output,
                   uint             imageW,
                   uint             imageH,
                   float            scale,
                   float2           view_center,
                   Eigen::Vector2f *means,
                   Eigen::Matrix2f *information_matrices,
                   int              N,
                   float            normalization,
                   float            tstep) {
  d_render<<<gridSize, blockSize>>>(
      d_output, imageW, imageH, scale, view_center, means, information_matrices, N, normalization, tstep);
}

#endif  // #ifndef _DIST_KERNEL_CU_
