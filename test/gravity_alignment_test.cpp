#include <cmath>
#include <cstdlib>
#include <iostream>

#include <Eigen/Geometry>

#include "gravity_alignment.hpp"

namespace
{

void require(const bool condition, const char * message)
{
  if (!condition)
  {
    std::cerr << "FAILED: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

}  // namespace

int main()
{
  constexpr double pi = 3.14159265358979323846;
  fast_lio::GravityAlignmentConfig config;
  config.min_duration_sec = 2.0;
  config.min_samples = 300U;
  config.max_samples = 1000U;
  config.max_gyro_norm_rad_s = 0.08;
  config.accel_norm_tolerance_m_s2 = 0.60;
  config.max_accel_std_m_s2 = 0.25;
  config.max_gyro_std_rad_s = 0.02;

  fast_lio::StationaryImuInitializer initializer(config);

  const Eigen::Matrix3d expected_world_from_imu =
    (Eigen::AngleAxisd(4.5 * pi / 180.0, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(-2.0 * pi / 180.0, Eigen::Vector3d::UnitX())).toRotationMatrix();
  const Eigen::Vector3d measured_specific_force =
    expected_world_from_imu.transpose() *
    (config.gravity_magnitude_m_s2 * Eigen::Vector3d::UnitZ());

  require(
    !initializer.add_sample(
      0.0, measured_specific_force, Eigen::Vector3d(0.2, 0.0, 0.0)),
    "moving sample must not initialize gravity");
  require(
    initializer.result().candidate_resets == 1U,
    "moving sample must reset the stationary candidate");

  for (std::size_t index = 0U; index < 500U; ++index)
  {
    const double timestamp = 1.0 + 0.005 * static_cast<double>(index);
    const double perturbation = 0.002 * std::sin(static_cast<double>(index));
    const Eigen::Vector3d acceleration =
      measured_specific_force + Eigen::Vector3d(perturbation, -perturbation, 0.0);
    initializer.add_sample(timestamp, acceleration, Eigen::Vector3d::Zero());
  }

  require(initializer.ready(), "stationary interval must initialize gravity");
  const fast_lio::GravityAlignmentResult & result = initializer.result();
  const Eigen::Vector3d aligned_force =
    result.rotation_world_from_imu * result.mean_accel_imu.normalized();
  require(
    (aligned_force - Eigen::Vector3d::UnitZ()).norm() < 1.0e-10,
    "mean specific force must map to +Z");
  require(
    (result.gravity_world - Eigen::Vector3d(0.0, 0.0, -config.gravity_magnitude_m_s2)).norm() <
    1.0e-12,
    "world gravity must be exactly -Z");
  require(
    std::abs(result.rotation_world_from_imu.determinant() - 1.0) < 1.0e-12,
    "alignment must be a proper rotation");
  require(result.duration_sec >= config.min_duration_sec, "minimum duration must be enforced");
  require(result.samples >= config.min_samples, "minimum sample count must be enforced");

  std::cout << "gravity_alignment_test passed\n";
  return EXIT_SUCCESS;
}
