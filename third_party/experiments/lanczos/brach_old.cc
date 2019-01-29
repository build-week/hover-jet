Eigen::VectorXd sigmoid_blend(const Eigen::VectorXd &x,
                              const double alpha,
                              const double beta) {
  constexpr double scale = 25.0;
  const Eigen::VectorXd x_exp = (scale * x).array().exp();
  const Eigen::VectorXd t = x_exp.array() / (x_exp.array() + 1.0);

  const Eigen::VectorXd convex_blend =
      ((1.0 - t.array()) * alpha) + (t.array() * beta);

  return convex_blend;
}

// Eigen::VectorXd inverse_sigmoid(const Eigen::VectorXd &x_sig,
//                                 const double alpha,
//                                 const double beta) {
//   const double t = (x.array() - alpha) / (beta - alpha);
//   const double x = (-t.array() / (t.array() - 1.0)).log();
//   return x;
// }

double cost_function(const Eigen::VectorXd &y,
                     Eigen::VectorXd *const gradient,
                     Eigen::MatrixXd *const hessian) {
  const double g = 1.0;
  const double dx = 0.001;
  const double alpha = static_cast<double>(y.rows()) * (dx + 0.1);

  const auto true_cost = [alpha, g, dx](const Eigen::VectorXd &x_un) {
    // Force y to be positive
    const Eigen::VectorXd x = sigmoid_blend(x_un, 0.0, alpha);
    Eigen::VectorXd dy_dx(x.rows() + 1);
    Eigen::VectorXd x_augmented(x.rows() + 1);
    x_augmented.segment(0, x.rows()) = x;
    x_augmented(x.rows()) = 0.0;

    // dy_dx.segment(1, n_diff_elements) =
    // (x.segment(1, n_diff_elements) - x.segment(0, n_diff_elements)) / dx;
    // dy_dx(0) = (x(0) - alpha) / dx;
    // dy_dx.tail(1) = -x.tail(1) / dx;

    dy_dx(0) = x_augmented(0) - alpha;
    for (int k = 1; k < x_augmented.rows(); ++k) {
      dy_dx(k) = x_augmented(k) - x_augmented(k - 1);
    }

    dy_dx.array() /= dx;

    const Eigen::VectorXd cost_numerators = 1.0 + dy_dx.array().square();
    const Eigen::VectorXd cost_denominators = alpha - x_augmented.array();

    if (cost_denominators.minCoeff() < 0) {
      std::cout << "uh oh" << std::endl;
      std::cout << cost_denominators.transpose() << std::endl;
      std::abort();
    }

    const Eigen::VectorXd times =
        (cost_numerators.array() / cost_denominators.array()).cwiseSqrt();
    const double total_time = (1.0 / std::sqrt(2.0 * g)) * times.sum() * dx;

    std::cout << "\n\n" << std::endl;
    std::cout << "y: " << x_augmented.transpose() << std::endl;
    std::cout << "dydx--" << std::endl;
    std::cout << dy_dx.transpose() << std::endl;
    std::cout << "num--" << std::endl;
    std::cout << cost_numerators.transpose() << std::endl;
    std::cout << "den--" << std::endl;
    std::cout << cost_denominators.transpose() << std::endl;
    std::cout << "time--" << std::endl;
    std::cout << times.transpose() << std::endl;
    std::cout << "--" << std::endl;

    return total_time;
  };

  if (gradient) {
    *gradient = numerics::dynamic_numerical_gradient(y, true_cost);

    std::cout << gradient->transpose() << std::endl;

    {
      const auto view = gl_viewer::get_window3d("soup");
      const auto geo = view->add_primitive<gl_viewer::SimpleGeometry>();
      const Eigen::VectorXd true_y = sigmoid_blend(y, 0.0, alpha);
      for (int k = 1; k < y.rows(); ++k) {
        const double x0 = (k - 1) * 0.1;
        const double x1 = k * 0.1;
        const Eigen::Vector3d start(x0, 0.0, true_y(k - 1));
        const Eigen::Vector3d end(x1, 0.0, true_y(k));
        geo->add_line({start, end});

        const double grad = (*gradient)(k);
        const double normalized_grad = grad * 0.05;
        const Eigen::Vector3d grad_end(x1, 0.0, true_y(k) + normalized_grad);
        geo->add_line({end, grad_end, Eigen::Vector4d(1.0, 0.0, 0.5, 0.5)});
      }

      view->spin_until_step();
      view->clear();
    }
  }

  return true_cost(y);
}
