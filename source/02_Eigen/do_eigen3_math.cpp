#include <eigen3/Eigen/Eigen>
#include <iostream>

int main(void) {
  // 1. Create two Eigen::Vector3d objects
  Eigen::Vector3d v1 = Eigen::Vector3d::Constant(10.0);
  Eigen::Vector3d v2(0.1, 0.5, 0.7);

  std::cout << "v1 = " << v1 << std::endl;
  std::cout << "v2 = " << v2 << std::endl;

  // 2. Create two Eigen::Matrix3d objects
  Eigen::Matrix3d m1 = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d m2;
  m2 << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;

  std::cout << "m1 = " << m1 << std::endl;
  std::cout << "m2 = " << m2 << std::endl;

  // 3. Operations:
  std::cout << "Scalar product v1*v2 = " << v1.transpose() * v2 << std::endl;
  std::cout << "(m1 +m2)*v2 = " << (m1 + m2) * v2 << std::endl;
  std::cout << "m1*m2 = " << m1 * m2 << std::endl;

  // 4. Element-wise operations: use array() class or .array() method of Vector and Matrix
  std::cout << "m1.*m2 = " << m1.array() * m2.array() << std::endl;
  std::cout << "rowwise max(m1 - m2) = " << (m1 - m2).array().rowwise().maxCoeff() << std::endl;

  // 5. First two elements:
  std::cout << "v1(1:2) = " << v1.head<2>() << std::endl;
  std::cout << "v1(1:2,1) = " << v1.col(0).head(2) << std::endl;

  std::cout << "v1(1:2 := [-pi, pi/4]" << std::endl;
  v1.head<2>() = Eigen::Vector2d(-M_PI, M_PI_4);
  std::cout << "v1 = " << v1 << std::endl;

  std::cout << "The program works" << std::endl;
  return 0;
}