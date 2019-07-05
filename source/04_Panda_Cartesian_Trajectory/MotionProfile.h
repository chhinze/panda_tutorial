#pragma once

#include <algorithm>
#include <eigen3/Eigen/Eigen>

/** \brief 3 times differentiable polynomial reference trajectory z(t) \in [0,1]
 *
 * z(t) = a_4*(t/T)^4 + a_5*(t/T)^5 +a_6*(t/T)^6 + a_7*(t/T)^7
 *
 * with a_4 = 35, a_5 = -84, a_6 = 70, a_7 = -20
 *
 * */
class PolynomialReferenceTraj {
private:
  /// the coefficients of the polynomial (a_4 - a_7)
  static const Eigen::Vector4d poly_coeffs;

  /// the factors obtained by 1st derivative of the polynomial exponents
  static const Eigen::Vector4d a_d_mult;

  // static const Eigen::Vector4d a_dd_mult(3,4,5,6);

  static constexpr double p_v_max = 2.1875;
  static constexpr double p_a_max = 7.513188404399293;

  /** \brief returns the end time of the motion profile depending on curve length L and allowed
   * dynamics limits
   * */
  static double get_t_E(double v_max_allowed, double a_max_allowed, double L, double dt);

public:
  /** \brief Evaluates the motion profile for the whole trajectory with normalized length coordinate
   * s \in [0,1]
   *
   * \arg[in] v_max_allowed maximum allowed velocity [m/s], the trajectory is adapted to this limit
   * \arg[in] a_max_allowed maximum allowed acceleration [m/s^2], the trajectory is adapted to this
   * limit \arg[in] L the real length of the calling curve p(s) [m] \arg[in] dt time step width [s]
   *
   * \returns std::pair containing generated time vector (1st element) and the position values (2nd
   * element)
   *
   * */
  static std::pair<Eigen::VectorXd, Eigen::VectorXd> s_t(double v_max_allowed, double a_max_allowed,
                                                         double L, double dt);

  /** \brief Evaluates the derivative of the motion profile for the whole trajectory w.r.t the
   * normalized length coordinate s \in [0,1]
   *
   * \arg[in] v_max_allowed maximum allowed velocity [m/s], the trajectory is adapted to this limit
   * \arg[in] a_max_allowed maximum allowed acceleration [m/s^2], the trajectory is adapted to this
   * limit \arg[in] L the real length of the calling curve [m] \arg[in] dt time step width [s]
   *
   * \returns std::pair containing generated time vector (1st element) and the  velocity values (2nd
   * element)
   *
   * */
  static std::pair<Eigen::VectorXd, Eigen::VectorXd>
  ds_dt(double v_max_allowed, double a_max_allowed, double L, double dt);
};

/** \brief Motion profile s(t) with smooth derivatives up to order 3.
 *
 * All functionality is initialized on construction, afterwards, only get-Access to members is
 * possible.
 *
 * */
class MotionProfile {
private:
  double L;
  double v_max;
  double a_max;
  double dt;

  Eigen::VectorXd t;
  Eigen::VectorXd s_t;
  Eigen::VectorXd ds_dt;

public:
  MotionProfile(double L, double v_max, double a_max, double dt);

  const Eigen::VectorXd &getTime() const;

  const Eigen::VectorXd &getS() const;

  const Eigen::VectorXd &getDsDt() const;

  double getL() const;
  double getV_max() const;
  double getA_max() const;
  double getDt() const;
  std::size_t getNumElements() const;
};