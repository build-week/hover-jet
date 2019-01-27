#include "numerics/jacobi_laplacian.hh"

#include "testing/gtest.hh"

namespace numerics {

TEST(JacobiLaplacianImage, test_converges) {
  //
  //
  //

  constexpr int ROWS = 50;
  using Mat = SquareMatNd<ROWS>;

  const Mat x = Mat::Random();
  const Mat b = Mat::Random() * 0.3;

  const Mat result = jacobi_laplacian_image(x, b, {});

  std::cout << result << std::endl;
}
}  // namespace numerics