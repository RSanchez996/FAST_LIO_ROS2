#ifndef FAST_LIO_GRAVITY_ALIGNMENT_HPP
#define FAST_LIO_GRAVITY_ALIGNMENT_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace fast_lio
{

struct GravityAlignmentConfig
{
  bool enabled = true;
  bool require_stationary = true;
  double min_duration_sec = 3.0;
  std::size_t min_samples = 400U;
  std::size_t max_samples = 4000U;
  double max_gyro_norm_rad_s = 0.08;
  double accel_norm_tolerance_m_s2 = 0.60;
  double max_accel_std_m_s2 = 0.25;
  double max_gyro_std_rad_s = 0.02;
  double gravity_magnitude_m_s2 = 9.80665;
  // Converts the acceleration values carried by sensor_msgs/Imu to m/s^2
  // before validation and filter initialization. ROS-compliant IMUs use 1.0;
  // livox_ros_driver2 publishes MID-360 acceleration in g, so use 9.80665.
  double input_accel_scale_to_m_s2 = 1.0;
};

struct GravityAlignmentResult
{
  bool ready = false;
  Eigen::Vector3d mean_accel_imu = Eigen::Vector3d::Zero();
  Eigen::Vector3d mean_gyro_imu = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_std_imu = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_std_imu = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rotation_world_from_imu = Eigen::Matrix3d::Identity();
  Eigen::Vector3d gravity_world = Eigen::Vector3d(0.0, 0.0, -9.80665);
  std::size_t samples = 0U;
  std::size_t rejected_samples = 0U;
  std::size_t candidate_resets = 0U;
  double duration_sec = 0.0;
  double max_gyro_norm_rad_s = 0.0;
  double latest_accel_norm_m_s2 = 0.0;
  double latest_gyro_norm_rad_s = 0.0;
  std::string status = "waiting_for_imu";
};

// Collects a contiguous stationary IMU interval and determines the unique
// minimum rotation that maps the measured specific-force direction to +Z.
// Gravity cannot determine yaw, so this rotation intentionally introduces no
// yaw beyond the minimum vector-to-vector alignment.
class StationaryImuInitializer
{
public:
  explicit StationaryImuInitializer(
    const GravityAlignmentConfig & config = GravityAlignmentConfig())
  : config_(config)
  {
    reset();
  }

  void configure(const GravityAlignmentConfig & config)
  {
    config_ = config;
    reset();
  }

  void reset()
  {
    samples_.clear();
    result_ = GravityAlignmentResult{};
    result_.gravity_world =
      Eigen::Vector3d(0.0, 0.0, -config_.gravity_magnitude_m_s2);
    last_timestamp_sec_ = -std::numeric_limits<double>::infinity();
  }

  bool add_sample(
    const double timestamp_sec,
    const Eigen::Vector3d & acceleration_imu,
    const Eigen::Vector3d & angular_velocity_imu)
  {
    if (result_.ready)
    {
      return true;
    }

    if (
      !std::isfinite(timestamp_sec) || !acceleration_imu.allFinite() ||
      !angular_velocity_imu.allFinite())
    {
      ++result_.rejected_samples;
      result_.status = "rejected_non_finite_imu";
      return false;
    }

    if (timestamp_sec <= last_timestamp_sec_)
    {
      // Synchronizers may expose the same boundary IMU sample in two adjacent
      // LiDAR groups. Ignore exact/older timestamps instead of weighting them
      // twice in the gravity estimate.
      ++result_.rejected_samples;
      result_.status = "ignored_duplicate_or_old_timestamp";
      return false;
    }
    last_timestamp_sec_ = timestamp_sec;

    const double gyro_norm = angular_velocity_imu.norm();
    result_.latest_accel_norm_m_s2 = acceleration_imu.norm();
    result_.latest_gyro_norm_rad_s = gyro_norm;
    const double accel_norm_error = std::abs(
      acceleration_imu.norm() - config_.gravity_magnitude_m_s2);

    if (
      config_.require_stationary &&
      (gyro_norm > config_.max_gyro_norm_rad_s ||
      accel_norm_error > config_.accel_norm_tolerance_m_s2))
    {
      samples_.clear();
      ++result_.rejected_samples;
      ++result_.candidate_resets;
      result_.status = gyro_norm > config_.max_gyro_norm_rad_s ?
        "waiting_stationary_gyro" : "waiting_stationary_accel_norm";
      update_progress();
      return false;
    }

    samples_.push_back(Sample{timestamp_sec, acceleration_imu, angular_velocity_imu});
    while (samples_.size() > config_.max_samples)
    {
      samples_.pop_front();
    }

    update_progress();
    if (
      samples_.size() < config_.min_samples ||
      result_.duration_sec < config_.min_duration_sec)
    {
      result_.status = "collecting_stationary_imu";
      return false;
    }

    compute_statistics();
    if (config_.require_stationary)
    {
      if (result_.accel_std_imu.maxCoeff() > config_.max_accel_std_m_s2)
      {
        restart_candidate_with_latest("waiting_stationary_accel_std");
        return false;
      }
      if (result_.gyro_std_imu.maxCoeff() > config_.max_gyro_std_rad_s)
      {
        restart_candidate_with_latest("waiting_stationary_gyro_std");
        return false;
      }
    }

    if (result_.mean_accel_imu.norm() < 1.0e-6)
    {
      restart_candidate_with_latest("invalid_zero_mean_acceleration");
      return false;
    }

    Eigen::Quaterniond world_from_imu = Eigen::Quaterniond::FromTwoVectors(
      result_.mean_accel_imu.normalized(), Eigen::Vector3d::UnitZ());
    world_from_imu.normalize();
    result_.rotation_world_from_imu = world_from_imu.toRotationMatrix();
    result_.gravity_world =
      Eigen::Vector3d(0.0, 0.0, -config_.gravity_magnitude_m_s2);
    result_.ready = true;
    result_.status = "gravity_aligned";
    return true;
  }

  const GravityAlignmentConfig & config() const {return config_;}
  const GravityAlignmentResult & result() const {return result_;}
  bool ready() const {return result_.ready;}

private:
  struct Sample
  {
    double timestamp_sec;
    Eigen::Vector3d acceleration_imu;
    Eigen::Vector3d angular_velocity_imu;
  };

  void update_progress()
  {
    result_.samples = samples_.size();
    result_.duration_sec = samples_.size() > 1U ?
      samples_.back().timestamp_sec - samples_.front().timestamp_sec : 0.0;
  }

  void compute_statistics()
  {
    Eigen::Vector3d acceleration_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity_sum = Eigen::Vector3d::Zero();
    double maximum_gyro_norm = 0.0;

    for (const Sample & sample : samples_)
    {
      acceleration_sum += sample.acceleration_imu;
      angular_velocity_sum += sample.angular_velocity_imu;
      maximum_gyro_norm = std::max(
        maximum_gyro_norm, sample.angular_velocity_imu.norm());
    }

    const double inverse_count = 1.0 / static_cast<double>(samples_.size());
    result_.mean_accel_imu = acceleration_sum * inverse_count;
    result_.mean_gyro_imu = angular_velocity_sum * inverse_count;
    result_.max_gyro_norm_rad_s = maximum_gyro_norm;

    Eigen::Vector3d acceleration_squared_error = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity_squared_error = Eigen::Vector3d::Zero();
    for (const Sample & sample : samples_)
    {
      const Eigen::Vector3d acceleration_error =
        sample.acceleration_imu - result_.mean_accel_imu;
      const Eigen::Vector3d angular_velocity_error =
        sample.angular_velocity_imu - result_.mean_gyro_imu;
      acceleration_squared_error += acceleration_error.cwiseProduct(acceleration_error);
      angular_velocity_squared_error +=
        angular_velocity_error.cwiseProduct(angular_velocity_error);
    }

    const double variance_denominator =
      samples_.size() > 1U ? static_cast<double>(samples_.size() - 1U) : 1.0;
    result_.accel_std_imu =
      (acceleration_squared_error / variance_denominator).cwiseSqrt();
    result_.gyro_std_imu =
      (angular_velocity_squared_error / variance_denominator).cwiseSqrt();
    update_progress();
  }

  void restart_candidate_with_latest(const std::string & status)
  {
    const Sample latest = samples_.back();
    samples_.clear();
    samples_.push_back(latest);
    ++result_.candidate_resets;
    result_.status = status;
    update_progress();
  }

  GravityAlignmentConfig config_;
  GravityAlignmentResult result_;
  std::deque<Sample> samples_;
  double last_timestamp_sec_ = -std::numeric_limits<double>::infinity();
};

}  // namespace fast_lio

#endif  // FAST_LIO_GRAVITY_ALIGNMENT_HPP
