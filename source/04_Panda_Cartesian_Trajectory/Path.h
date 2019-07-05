#pragma once

#include <eigen3/Eigen/Eigen>

#include <algorithm>
#include <stdexcept>

namespace Eigen {
/// 6d vector representation (used for [x,y,z,roll_x,pitch_y,yaw_z])
typedef Matrix<double, 6, 1> Vector6d;

/// Matrix with 6 rows stored colwise (fast colwise access)
typedef Eigen::Matrix<double, 6, Eigen::Dynamic, Eigen::ColMajor> Matrix6dynd;
} // namespace Eigen

/** \brief Abstract superclass for all path objects. Defines the interface for all concrete paths.
 */
class PathAnalytical {
public:
  virtual ~PathAnalytical() {}

  /** \brief Evaluate a path p(s) at a given coordinate `s` and return the (possibly
   * multi-dimensional) values
   *
   * \arg[in] s the path coordinate ∈[0,1] if s is >1, then it is forced to be 1, if it is <0,
   * then it is forced to 0.
   *
   * \returns the path evaluated at s.
   */
  virtual Eigen::Vector6d at(double s) = 0;

  virtual Eigen::Matrix6dynd at(const Eigen::VectorXd &s) = 0;

  /** \brief Return the derivative w.r.t s of the path at the given coordinate s.
   * */
  virtual Eigen::Vector6d ds_at(double s) = 0;

  virtual Eigen::Matrix6dynd ds_at(const Eigen::VectorXd &s) = 0;

  /** \brief Returns a length of the analytical path [m].
   * */
  virtual double getL() = 0;
};

class LinearPath : public PathAnalytical {
  Eigen::Vector6d from;
  Eigen::Vector6d direction;

  Eigen::VectorXd clamp(const Eigen::VectorXd &v, double lowerLimit, double upperLimit);

public:
  LinearPath(const Eigen::Vector6d &from, const Eigen::Vector6d &to);

  Eigen::Vector6d at(double s) override;

  virtual Eigen::Matrix6dynd at(const Eigen::VectorXd &s) override;

  Eigen::Vector6d ds_at(double) override;

  virtual Eigen::Matrix6dynd ds_at(const Eigen::VectorXd &s) override;

  std::size_t dim();

  double getL() override;
};