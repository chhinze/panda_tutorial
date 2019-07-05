#pragma once

#include "Path.h"
#include "MotionProfile.h"

#include <franka/control_types.h>
#include <franka/robot_state.h>
#include <franka/duration.h>

#include <memory>
#include <array>

/** \brief convert Franka's homogeneous transformation (std::array) to a 6D pose with [x,y,z,
 * roll,pitch,yaw].
 *
 * \arg[in] 16 dimensional array (flattened homogeneous transformation)
 * \returns 6d pose vector with 3 cartesian positions and 3 rpy rotations (r_x, p_y, y_z)
 *
 * \warning The rotations are constrained to get a reproducible construction, i.e.
 *          roll_x \in [0,π], pitch_y \in [-π, π], yaw_z \in [-π, π].
 *
 * \todo change rotations to less constrained ones
 */
Eigen::Vector6d homogeneousTfArray2PoseVec(const std::array<double, 16> &pose_TF_as_array);

/** \brief convert a 6D pose with [x,y,z, roll,pitch,yaw] to Franka's homogeneous transformation
 * (std::array)
 *
 * \arg[in] pose the 6 d pose (3 translations, 3 rpy rotations)
 * \returns 16 dimensional array (flattened homogeneous transformation)
 *
 * \warning The rotations are constrained to get a reproducible construction, i.e.
 *          roll_x \in [0,π], pitch_y \in [-π, π], yaw_z \in [-π, π].
 *
 * \todo change rotations to less constrained ones
 */
std::array<double, 16> poseVec2HomogeneousTfArray(const Eigen::Vector6d &pose);

/** \brief base class for a cartesian trajectory */
class Trajectory {
public:
  virtual ~Trajectory() {}

  /** \brief interface to get the pose values column-wise for the whole trajectory, sampled with dt
   */
  virtual Eigen::Matrix6dynd p_t() const = 0;

  /** \brief interface to get the pose velocity values column-wise for the whole trajectory, sampled
   * with dt */
  virtual Eigen::Matrix6dynd dp_dt() const = 0;

  /** \brief interface to get sample period dt [s] */
  virtual double getDt() const = 0;

  /** \brief interface to get calculated end time [s] */
  virtual double getTEnd() const = 0;
};

/** \brief implements a cartesian linear trajectory between two points.
 *
 * The motion is determined via a polynomial reference trajectory, which is guaranteed to keep the
 * given dynamics limits.
 *
 * \note The trajectory is not time-optimal w.r.t. the dynamics limits.
 *
 * \warning: jerk limits are not yet implemented.
 */
class LinearTrajectory : public Trajectory {
protected:
  /// the geometric description of the path
  std::unique_ptr<LinearPath> p_s;

  /// the polynomial mapping between path coordinate and time.
  std::unique_ptr<MotionProfile> s_t;

public:
  /** \brief create a linear trajectory witgh motion profile between to points (end effector in
   * robot base coordinate system).
   *
   * \arg[in] from starting pose (could be obtained from the robot directly before creation)
   *          [x,y,z, R, P, Y], position in [m], orientation in [rad]
   * \arg[in] to goal position [x,y,z, R, P, Y]
   * \arg[in] v_max the maximum allowed velocity [m/s, rad/s]
   * \arg[in] a_max the maximum allowed acceleration [m/s², rad/s²]
   * \arg[in] j_max the maximum allowed jerk [m/s³, rad/s³]
   * (**This is not yet kept.**)
   * \arg[in] dt the discretization step width [s]. Must match the execution times in the robot
   * control.
   */
  LinearTrajectory(const Eigen::Vector6d &from, const Eigen::Vector6d &to, double v_max,
                   double a_max, double dt);

  /** \brief get the pose values column-wise for the whole trajectory (end effector in robot base),
   * sampled with dt. */
  Eigen::Matrix6dynd p_t() const override;

  /** \brief get the pose velocity values column-wise for the whole trajectory (end effector in
   * robot base), sampled with dt. */
  Eigen::Matrix6dynd dp_dt() const override;

  /** \brief get sample period dt [s] */
  double getDt() const override;

  /** \brief get calculated end time [s] */
  double getTEnd() const override;
};

/** \brief General iterator for cartesian paths, providing methods for obtaining pose, poseVelovity
 * and iterate one time step.
 *
 *  This can be used to manually build a position or velocity trajectory for the robot. For the
 * Franka Emika Panda it is easier to use TrajectoryIteratorCartesianVelocity.
 */
class TrajectoryIteratorCartesian {
private:
  /// column-wise storage of precalculated poses for the whole trajectory (end effector in
  /// robot base coordinate system).
  Eigen::Matrix6dynd p_t;

  /// column-wise storage of precalculated pose velocities for the whole trajectory (end effector in
  /// robot base coordinate system).
  Eigen::Matrix6dynd dp_dt;

  /// step size between time steps [s]
  const double dt;

  // end time of the trajcectory. Assuming it starts at 0 and progresses constantly with dt [s]
  const double t_E;

  /// current iteration number
  Eigen::Index itr;

public:
  TrajectoryIteratorCartesian(const Trajectory &traj);

  /** \brief get current cartesian pose (x,y,z, R,P,Y).
   *
   * \returns the current cartesian pose as homogeneous transformation matrix (16 double values)
   *
   * \note the return value will be the same, until step() is called.
   *
   */
  std::array<double, 16> getCartesianPose() const;

  /** \brief get current cartesian velocities
   *
   * \returns the current velocities (v_x, v_y, v_z, omega_x, omega_x, omega_z), v in [m/s], omega
   * in [rad/s].
   *
   * \note To iterate over the dataset, call step(). Otherwise, the same value is fetched every
   * time.
   * */
  std::array<double, 6> getCartesianVelocity() const;

  /** \brief iterate to the next time instance.
   *
   * the internal pointer is counted up, pointing to the next instance for pose and velocities.
   */
  void step();

  /** \brief get the current time [s], which is calculated from the number of steps taken and the
   * step width dt.
   *
   * \returns the current time [s].  */
  double getCurrentTime() const;

  /** \brief get the precalculated end time of the current trajectory.
   *
   * This can be used to compare, if the tajectory is finished already, i.e.
   *
   * ```{.cpp}
   * if (getCurrentTime() <= getEndTime()){
   *  std::cout<<"We are before the trajectory's end."<<std::endl;
   * }else{
   *  std::cout<<"End of trajectory is reached."<<std::endl;
   * }
   * ```
   * \returns calculated end time [s].
   *
   */
  double getEndTime() const;
};

/** \brief defines the controlller callback for speed control directly with `operator()`.
 *
 * This makes it possible to directly call
 *
 * ```{.cpp}
 * franka::Robot robot = ... ;
 * TrajectoryIteratorCartesianVelocity vObj = ...;
 * robot.contol(vObj);
 * ```
 *
 * */
class TrajectoryIteratorCartesianVelocity : public TrajectoryIteratorCartesian {
public:
  /** \brief Build a new TrajectoryIteratorCartesianVelocity for using it with `libfranka`.
   *
   * The desired velocities are extracted from the given `Trajectory`.
   *
   * \arg[in] traj The trajectory object, describing the cartesian toolpath.
   */
  TrajectoryIteratorCartesianVelocity(const Trajectory &traj) : TrajectoryIteratorCartesian(traj) {}

  /** \brief function call interface `(const franka::RobotState&, franka::Duration) ->
   * franka::CartesianVelocities`, which can be directly used as velocities trajectory in
   * `libfranka`'s control (matching the velocity interface).
   *
   * The internal time pointer is advanced in each call to this function.
   *
   * \note The trajectory is passed offline, so neither the RobotState, nor the Duration
   * is used.
   *
   * \returns franka::CartesianVelocities (3 translational and 3 rotational velocities) for each
   * time step.
   *
   */
  franka::CartesianVelocities operator()(const franka::RobotState &, franka::Duration);
};
