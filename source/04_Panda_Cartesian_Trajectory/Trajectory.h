#pragma once

#include "Path.h"
#include "MotionProfile.h"

#include <memory>
#include <array>

/** convert Franka's homogeneous transformation (std::array) to a 6D pose with [x,y,z,
 * roll,pitch,yaw].
 *
 * \arg 16 dimensional array (flattened homogeneous transformation)
 * \returns 6d pose vector with 3 cartesian positions and 3 rpy rotations (r_x, p_y, y_z)
 *
 * \warning The rotations are constrained to get a reproducible construction, i.e.
 *          roll_x \in [0,\pi], pitch_y \in [-\pi, \pi], yaw_z \in [-\pi, \pi].
 *
 * \todo change rotations to less constrained ones
 */
Eigen::Vector6d homogeneousTfArray2PoseVec(const std::array<double, 16> &pose_TF_as_array);

/** convert a 6D pose with [x,y,z, roll,pitch,yaw] to Franka's homogeneous transformation
 * (std::array)
 *
 * \arg pose the 6 d pose (3 translations, 3 rpy rotations)
 * \returns 16 dimensional array (flattened homogeneous transformation)
 *
 * \warning The rotations are constrained to get a reproducible construction, i.e.
 *          roll_x \in [0,\pi], pitch_y \in [-\pi, \pi], yaw_z \in [-\pi, \pi].
 *
 * \todo change rotations to less constrained ones
 */
std::array<double, 16> poseVec2HomogeneousTfArray(const Eigen::Vector6d &pose);

class Trajectory {
public:
  virtual ~Trajectory() {}
  virtual Eigen::Matrix6dynd p_t() const = 0;
  virtual Eigen::Matrix6dynd dp_dt() const = 0;

  virtual double getDt() const = 0;
  virtual double getTEnd() const = 0;
};

class LinearTrajectory : public Trajectory {
protected:
  std::unique_ptr<LinearPath> p_s;
  std::unique_ptr<MotionProfile> s_t;

public:
  LinearTrajectory(const Eigen::Vector6d &from, const Eigen::Vector6d &to, double v_max,
                   double a_max, double dt);

  Eigen::Matrix6dynd p_t() const override;

  Eigen::Matrix6dynd dp_dt() const override;

  double getDt() const override;

  double getTEnd() const override;
};

class TrajectoryIteratorCartesianPose {
private:
  Eigen::Matrix6dynd p_t, dp_dt;

  /// step sitew between time steps
  const double dt;

  // end time of the trajcectory. Assuming it starts at 0 and progresses constantly with dt
  const double t_E;

  /// current iteration
  std::size_t itr;

public:
  TrajectoryIteratorCartesianPose(const Trajectory &traj);

  std::array<double, 16> getCartesianPose() const;

  /** Returns the current velocities (v_x, v_y, v_z, omega_x, omega_x, omega_z), v in [m/s], omega in [rad/s].
   * 
   * \note To iterate over the dataset, call step(). Otherwise, the same value is fetched every time.
   * */
  std::array<double, 6> getCartesianVelocity() const;

  void step();

  double getCurrentTime() const;
  double getEndTime() const;
};
