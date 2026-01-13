#include <iostream>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "smm_screws/core/ScrewsMain.h"

int main()
{
  ScrewsMain sm;

  // --------------------------------------------------
  // 1) Build a simple revolute twist about z through origin
  //    ω = [0 0 1]^T, v = -ω × q, with q = 0 => v = 0
  // --------------------------------------------------
  Eigen::Vector3f omega(0.0f, 0.0f, 1.0f);
  Eigen::Vector3f v(0.0f, 0.0f, 0.0f);

  Eigen::Matrix<float, 6, 1> xi;
  sm.formTwist(xi, v, omega);   // xi = [v; ω]

  std::cout << "Twist xi = [v; w] =\n" << xi << "\n\n";

  // --------------------------------------------------
  // 2) Exponential map: g = exp( xi * theta )
  //    Rotate 45 degrees (pi/4) about z
  // --------------------------------------------------
  const float theta = static_cast<float>(M_PI) / 4.0f;  // 45 degrees

  Eigen::Isometry3f g = sm.twistExp(xi, theta);

  std::cout << "Homogeneous transform g = exp(xi * theta):\n"
            << g.matrix() << "\n\n";

  // --------------------------------------------------
  // 3) Another twist: rotate about y-axis
  //    just to demonstrate Lie bracket / screwProduct
  // --------------------------------------------------
  Eigen::Vector3f omega2(0.0f, 1.0f, 0.0f);
  Eigen::Vector3f v2(0.0f, 0.0f, 0.0f);

  Eigen::Matrix<float, 6, 1> xi2;
  sm.formTwist(xi2, v2, omega2);

  std::cout << "Second twist xi2 = [v2; w2] =\n" << xi2 << "\n\n";

  // Lie bracket [xi, xi2] via se(3) commutator
  Eigen::Matrix<float, 6, 1> lb = sm.lb(xi, xi2);
  std::cout << "Lie bracket [xi, xi2] =\n" << lb << "\n\n";

  // Mueller "screw product" version
  Eigen::Matrix<float, 6, 1> sp = sm.screwProduct(xi, xi2);
  std::cout << "Screw product ad(xi)*xi2 =\n" << sp << "\n\n";

  return 0;
}
