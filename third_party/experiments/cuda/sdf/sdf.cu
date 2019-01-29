#ifndef _DIST_KERNEL_CU_
#define _DIST_KERNEL_CU_

#include <helper_cuda.h>
#include <helper_math.h>

#define EIGEN_DEFAULT_DENSE_INDEX_TYPE int
#include <Eigen/Dense>

__device__ uint pack_frgba(float4 rgba) {
  rgba.x = __saturatef(rgba.x);  // clamp to [0.0, 1.0]
  rgba.y = __saturatef(rgba.y);
  rgba.z = __saturatef(rgba.z);
  rgba.w = __saturatef(rgba.w);
  return (uint(rgba.w * 255) << 24) | (uint(rgba.z * 255) << 16) |
         (uint(rgba.y * 255) << 8) | uint(rgba.x * 255);
}

__device__ float sd_sphere(const Eigen::Vector3f &center,
                           const float radius,
                           const Eigen::Vector3f &ray_dir,
                           const Eigen::Vector3f &ray_origin) {
  float min_dist = 1000.0f;
  for (float t = 0.0f; t < 10.0f; t += 0.1f) {
    const Eigen::Vector3f error = ((t * ray_dir) + ray_origin) - center;
    const float sd_sphere = error.norm() - radius;
    min_dist = fminf(min_dist, sd_sphere);
  }
  return min_dist;
}

__global__ void d_render(uint *const d_output,
                         const uint imageW,
                         const uint imageH,
                         const Eigen::Vector3f *const centers,
                         const float *const radii,
                         const int N,
                         const float normalization,
                         const float t) {
  const uint x = blockIdx.x * blockDim.x + threadIdx.x;
  const uint y = blockIdx.y * blockDim.y + threadIdx.y;

  if ((x >= imageW) || (y >= imageH)) {
    return;
  }

  // These int-float cast operations aren't cheap
  const float unscaled_u = ((x / (float)imageW) * 2.0f) - 1.0f;
  const float unscaled_v = ((y / (float)imageH) * 2.0f) - 1.0f;

  const float2 view_center = make_float2(0.0f, 0.0f);
  const float scale = 1.0;

  const float u = scale * (unscaled_u + view_center.x);
  const float v = scale * (unscaled_v + view_center.y);

  float max_dist = 10000.0f;
  const Eigen::Vector3f pixel_position(u, v, 0.0f);
  const Eigen::Vector3f pixel_ray = Eigen::Vector3f(u, v, 1.0f).normalized();

  const Eigen::AngleAxisf rot(t, Eigen::Vector3f::UnitZ());

  for (uint k = 0u; k < N; ++k) {
    const float distance =
        sd_sphere(rot * centers[k], radii[k], pixel_ray, pixel_position);
    max_dist = fminf(max_dist, distance);
    // max_dist = fmaxf(max_dist, distance);
  }

  float4 color;
  // color.x = max_dist * normalization;
  color.x = max_dist < 0.0f ? 1.0f : 0.0f;

  color.y = 0.0f;
  color.z = 0.0f;
  color.w = 1.0f;

  // This could be optimized
  d_output[y * imageW + x] = pack_frgba(color);
}

void render_kernel(dim3 gridSize,
                   dim3 blockSize,
                   uint *const d_output,
                   const uint imageW,
                   const uint imageH,
                   const Eigen::Vector3f *const centers,
                   const float *const radii,
                   const int N,
                   const float normalization,
                   const float t) {
  d_render<<<gridSize, blockSize>>>(
      d_output, imageW, imageH, centers, radii, N, normalization, t);
}

#endif  // #ifndef _DIST_KERNEL_CU_
