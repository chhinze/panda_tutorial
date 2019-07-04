#include "MotionProfile.h"

// the coefficients of the polynomial (a_4 - a_7)
const Eigen::Vector4d PolynomialReferenceTraj::poly_coeffs(35., -84., 70., -20.);

// the factors obtained by 1st derivative of the polynomial exponents
const Eigen::Vector4d PolynomialReferenceTraj::a_d_mult(4., 5., 6., 7.);

double PolynomialReferenceTraj::get_t_E(double v_max_allowed, double a_max_allowed, double L,
                                        double dt) {
  double t_E = std::max(p_v_max * L / v_max_allowed, sqrt(p_a_max * L / a_max_allowed)); // end time
  return ceil(t_E / dt) * dt; // round t_E to whole dt steps number
}

std::pair<Eigen::VectorXd, Eigen::VectorXd>
PolynomialReferenceTraj::s_t(double v_max_allowed, double a_max_allowed, double L, double dt) {
  if (v_max_allowed < 0 || a_max_allowed < 0 || L < 0 || dt < 0) {
    throw std::invalid_argument(
        "PolynomialReferenceTraj::s_t. Parameters are required to be positive.");
  }

  double t_E = get_t_E(v_max_allowed, a_max_allowed, L, dt);
  std::size_t N_poly = poly_coeffs.size();

  Eigen::VectorXd t =
      Eigen::VectorXd::LinSpaced(t_E / dt + 1, 0, t_E) / t_E; // generate normalized time vector
  Eigen::MatrixXd _T(N_poly, t.size());

  for (std::size_t i = 0; i < N_poly; i++) {
    double expCoeff = N_poly + i;
    _T.row(i) = t.array().pow(expCoeff);
  }

  Eigen::VectorXd s = poly_coeffs.transpose() * _T;
  t = t_E * t; // rescale time
  return std::make_pair(t, s);
}

std::pair<Eigen::VectorXd, Eigen::VectorXd>
PolynomialReferenceTraj::ds_dt(double v_max_allowed, double a_max_allowed, double L, double dt) {
  if (v_max_allowed < 0 || a_max_allowed < 0 || L < 0 || dt < 0) {
    throw std::invalid_argument(
        "PolynomialReferenceTraj::s_t. Parameters are required to be positive.");
  }

  double t_E = get_t_E(v_max_allowed, a_max_allowed, L, dt);
  std::size_t N_poly = poly_coeffs.size();

  Eigen::VectorXd t =
      Eigen::VectorXd::LinSpaced(t_E / dt + 1, 0, t_E) / t_E; // generate normalized time vector
  Eigen::MatrixXd _T(N_poly, t.size());

  for (std::size_t i = 0; i < N_poly; i++) {
    double expCoeff = N_poly + i - 1;
    _T.row(i) = t.array().pow(expCoeff);
  }

  Eigen::VectorXd deriv_poly_coeffs = poly_coeffs.cwiseProduct(a_d_mult) / t_E;

  Eigen::VectorXd ds = deriv_poly_coeffs.transpose() * _T;
  t = t_E * t; // rescale time
  return std::make_pair(t, ds);
}

MotionProfile::MotionProfile(double L, double v_max, double a_max, double dt)
    : L(L), v_max(v_max), a_max(a_max), dt(dt) {
  if (L <= 0 || v_max <= 0 || a_max <= 0 || dt <= 0) {
    throw std::invalid_argument("L, v_max, a_max and dt are physical properties, must be >0.");
  }

  auto pos = PolynomialReferenceTraj::s_t(v_max, a_max, L, dt);
  auto vel = PolynomialReferenceTraj::ds_dt(v_max, a_max, L, dt);

  t = pos.first;
  s_t = pos.second;
  ds_dt = vel.second;
}

const Eigen::VectorXd &MotionProfile::getTime() const { return t; }

const Eigen::VectorXd &MotionProfile::getS() const { return s_t; }

const Eigen::VectorXd &MotionProfile::getDsDt() const { return ds_dt; }

double MotionProfile::getL() const { return L; }
double MotionProfile::getV_max() const { return v_max; }
double MotionProfile::getA_max() const { return a_max; }
double MotionProfile::getDt() const { return dt; }

std::size_t MotionProfile::getNumElements() const { return this->s_t.size(); }