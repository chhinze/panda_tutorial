#include "Trajectory.h"

Eigen::Vector6d homogeneousTfArray2PoseVec(const std::array<double, 16> &pose_TF_as_array) {

  Eigen::Isometry3d TF = Eigen::Isometry3d::Identity();
  TF.matrix() = Eigen::Matrix<double, 4, 4, Eigen::ColMajor>(pose_TF_as_array.data());
  Eigen::Vector6d pose;
  pose << TF.translation(), TF.rotation().eulerAngles(0, 1, 2);
  return pose;
}

std::array<double, 16> poseVec2HomogeneousTfArray(const Eigen::Vector6d &pose) {
  if (pose(3) < 0 || pose(3) > M_PI || pose(4) < -M_PI || pose(4) > M_PI || pose(5) < -M_PI ||
      pose(5) > M_PI) {
    throw std::invalid_argument("Angle representation is currently constrained to [0,pi], "
                                "[-pi,pi], [-pi,pi] for last 3 elements of pose vector");
  }

  Eigen::Vector3d trans = pose.head(3);
  double roll = pose(3), pitch = pose(4), yaw = pose(5);
  Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

  // ZY'X'' convention, see https://en.wikipedia.org/wiki/Euler_angles#Conventions_2, ch. Tait–Bryan
  // angles
  Eigen::Quaterniond q = rollAngle * pitchAngle * yawAngle; // yawAngle * pitchAngle * rollAngle;

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = trans;
  tf.rotate(q);

  std::array<double, 16> v;
  Eigen::Matrix<double, 4, 4, Eigen::ColMajor>::Map(v.data()) = tf.matrix();

  return v;
}

LinearTrajectory::LinearTrajectory(const Eigen::Vector6d &from, const Eigen::Vector6d &to,
                                   double v_max, double a_max, double dt) {
  this->p_s = std::make_unique<LinearPath>(from, to);
  const double L = (to - from).head(3).norm();
  this->s_t = std::make_unique<MotionProfile>(L, v_max, a_max, dt);
}

Eigen::Matrix6dynd LinearTrajectory::p_t() const {
  const Eigen::VectorXd &s = s_t->getS();
  return p_s->at(s);
}

Eigen::Matrix6dynd LinearTrajectory::dp_dt() const {
  const Eigen::VectorXd &s = s_t->getS();
  const Eigen::VectorXd &ds_dt = s_t->getDsDt();
  return p_s->ds_at(s).cwiseProduct(Eigen::VectorXd::Ones(p_s->dim()) * ds_dt.transpose());
}

double LinearTrajectory::getDt() const { return this->s_t->getDt(); }

double LinearTrajectory::getTEnd() const { return this->s_t->getNumElements() * this->getDt(); }

TrajectoryIteratorCartesian::TrajectoryIteratorCartesian(const Trajectory &traj)
    : p_t(traj.p_t()), dp_dt(traj.dp_dt()), dt(traj.getDt()), t_E(traj.getTEnd()), itr(0) {}

std::array<double, 16> TrajectoryIteratorCartesian::getCartesianPose() const {
  const Eigen::Vector6d currentPose = this->p_t.col(this->itr);

  return poseVec2HomogeneousTfArray(currentPose);
}

std::array<double, 6> TrajectoryIteratorCartesian::getCartesianVelocity() const {
  const Eigen::Vector6d currentVel = this->dp_dt.col(this->itr);
  std::array<double, 6> retVal;
  Eigen::Vector6d::Map(retVal.data()) = currentVel;
  return retVal;
}

void TrajectoryIteratorCartesian::step() { itr = itr + 1; }

double TrajectoryIteratorCartesian::getCurrentTime() const { return this->itr * this->dt; }
double TrajectoryIteratorCartesian::getEndTime() const { return this->t_E; }

franka::CartesianVelocities TrajectoryIteratorCartesianVelocity::
operator()(const franka::RobotState &, franka::Duration) {
  auto cartesianVelDes = franka::CartesianVelocities(getCartesianVelocity());
  step();

  if (getCurrentTime() < getEndTime()) {
    return cartesianVelDes;
  } else {
    return franka::MotionFinished(cartesianVelDes);
  }
}