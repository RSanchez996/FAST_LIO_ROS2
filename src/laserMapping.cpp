// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <chrono>
#include <cctype>
#include <condition_variable>
#include <unistd.h>
#include <so3_math.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "IMU_Processing.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/time.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>
#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp/executors/multi_threaded_executor.hpp>

struct TemporalObservationPoint
{
    float x;
    float y;
    float z;
    float intensity;
    float time_offset_sec;
    std::uint32_t scan_id;
    double timestamp_sec;
};

static_assert(
    std::is_standard_layout<TemporalObservationPoint>::value,
    "TemporalObservationPoint must remain a standard-layout binary record");
static_assert(
    std::is_trivially_copyable<TemporalObservationPoint>::value,
    "TemporalObservationPoint must remain trivially copyable");
static_assert(offsetof(TemporalObservationPoint, x) == 0U);
static_assert(offsetof(TemporalObservationPoint, y) == 4U);
static_assert(offsetof(TemporalObservationPoint, z) == 8U);
static_assert(offsetof(TemporalObservationPoint, intensity) == 12U);
static_assert(offsetof(TemporalObservationPoint, time_offset_sec) == 16U);
static_assert(offsetof(TemporalObservationPoint, scan_id) == 20U);
static_assert(offsetof(TemporalObservationPoint, timestamp_sec) == 24U);
static_assert(
    sizeof(TemporalObservationPoint) == 32U,
    "TemporalObservationPoint PCD record must remain exactly 32 bytes");

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool   runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
/**************************/

std::vector<float> res_last;
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0;

std::mutex mtx_buffer;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic;

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int effct_feat_num = 0, time_log_counter = 0, scan_count = 0;
// Número de hilos OpenMP usados exclusivamente en la búsqueda
// de correspondencias punto-mapa.
int openmp_threads = 2;
int iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
std::vector<std::uint8_t> point_selected_surf;
bool lidar_pushed = false, flg_first_scan = true, flg_EKF_inited = false;
bool scan_pub_en = false;
bool dense_pub_en = false;

// Permiten controlar independientemente la nube global y la nube LiDAR local.
bool scan_world_pub_en = false;
bool scan_lidar_pub_en = true;
bool is_first_lidar = true;

vector<vector<int>>  pointSearchInd_surf; 
vector<BoxPointType> cub_needrm;
vector<PointVector>  Nearest_Points; 
vector<double>       extrinT(3, 0.0);
vector<double>       extrinR(9, 0.0);
deque<double> time_buffer;
deque<double> preprocess_time_buffer;
deque<PointCloudXYZI::Ptr> lidar_buffer;
deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;

double current_preprocess_time = 0.0;

struct QueueDiagnostics {
    std::uint64_t lidar_received = 0;
    std::uint64_t imu_received = 0;
    std::uint64_t synced_scans = 0;

    // Número de llamadas a sync_packages() en las que todavía
    // no había suficiente IMU para completar el scan LiDAR.
    std::uint64_t wait_for_imu_polls_total = 0;
    std::uint64_t wait_for_imu_polls_interval = 0;

    std::uint64_t lidar_timestamp_resets = 0;
    std::uint64_t imu_timestamp_resets = 0;

    std::size_t lidar_high_watermark_total = 0;
    std::size_t imu_high_watermark_total = 0;

    std::size_t lidar_high_watermark_interval = 0;
    std::size_t imu_high_watermark_interval = 0;
};

QueueDiagnostics queue_diagnostics;

// Scans que han completado deskew, registro, EKF,
// actualización del ikd-tree y publicaciones.
std::atomic<std::uint64_t> mapping_completed_count{0};

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE<PointType> ikdtree;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;

nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::Quaternion geoQuat;
geometry_msgs::msg::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

inline void dump_lio_state_to_log(FILE *fp)  
{
    if (fp == nullptr) return;
    
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2)); // Pos  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2)); // Vel  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));    // Bias_g  
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));    // Bias_a  
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a  
    fprintf(fp, "\r\n");  
}

void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}


void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

tf2::Transform make_lio_tracking_transform() {
  // Pose estimada internamente por FAST-LIO: mundo -> IMU.
  tf2::Transform T_lio_imu;

  T_lio_imu.setOrigin(tf2::Vector3(state_point.pos(0), state_point.pos(1), state_point.pos(2)));

  tf2::Quaternion q_lio_imu(state_point.rot.coeffs()[0], state_point.rot.coeffs()[1], state_point.rot.coeffs()[2], state_point.rot.coeffs()[3]);

  q_lio_imu.normalize();
  T_lio_imu.setRotation(q_lio_imu);

  // Extrínseca almacenada por FAST-LIO:
  //
  // p_imu = R_L_I * p_lidar + t_L_I
  //
  // Por tanto representa directamente IMU <- LiDAR.
  tf2::Transform T_imu_lidar;

  T_imu_lidar.setOrigin(tf2::Vector3(state_point.offset_T_L_I(0), state_point.offset_T_L_I(1), state_point.offset_T_L_I(2)));

  tf2::Quaternion q_imu_lidar(state_point.offset_R_L_I.coeffs()[0], state_point.offset_R_L_I.coeffs()[1], state_point.offset_R_L_I.coeffs()[2], state_point.offset_R_L_I.coeffs()[3]);

  q_imu_lidar.normalize();
  T_imu_lidar.setRotation(q_imu_lidar);

  // Mundo <- LiDAR.
  return T_lio_imu * T_imu_lidar;
}

void transform_lio_world_point_to_map(const PointType & point_lio_world, PointType * point_map, const tf2::Transform & T_map_lio) {
    const tf2::Vector3 p_map =
        T_map_lio * tf2::Vector3(
            point_lio_world.x,
            point_lio_world.y,
            point_lio_world.z);

    point_map->x = p_map.x();
    point_map->y = p_map.y();
    point_map->z = p_map.z();
    point_map->intensity = point_lio_world.intensity;
}

tf2::Transform make_transform_from_eigen(
    const M3D & rotation,
    const V3D & translation)
{
    Eigen::Quaterniond eigen_quaternion(rotation);
    eigen_quaternion.normalize();

    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(
        translation.x(),
        translation.y(),
        translation.z()));
    transform.setRotation(tf2::Quaternion(
        eigen_quaternion.x(),
        eigen_quaternion.y(),
        eigen_quaternion.z(),
        eigen_quaternion.w()));
    return transform;
}

void set_pose_from_transform(geometry_msgs::msg::Pose & pose, const tf2::Transform & transform) {
    pose.position.x = transform.getOrigin().x();
    pose.position.y = transform.getOrigin().y();
    pose.position.z = transform.getOrigin().z();
    pose.orientation = tf2::toMsg(transform.getRotation());
}

tf2::Transform project_transform_to_se2(const tf2::Transform & transform) {
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    tf2::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);

    tf2::Quaternion q_2d;
    q_2d.setRPY(0.0, 0.0, yaw);
    q_2d.normalize();

    tf2::Transform transform_2d;
    transform_2d.setOrigin(tf2::Vector3(
        transform.getOrigin().x(),
        transform.getOrigin().y(),
        0.0));
    transform_2d.setRotation(q_2d);

    return transform_2d;
}

void points_cache_collect() {
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment() {
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;    
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr msg) {
  const double current_timestamp = get_time_sec(msg->header.stamp);
  const double preprocess_start = omp_get_wtime();
  auto cloud = std::make_shared<PointCloudXYZI>();

  // El procesamiento costoso se realiza sin bloquear la IMU.
  p_pre->process(msg, cloud);
  const double preprocess_duration = omp_get_wtime() - preprocess_start;

  {
    std::lock_guard<std::mutex> lock(mtx_buffer);
    ++scan_count;

    if (!is_first_lidar && current_timestamp < last_timestamp_lidar) {
      RCLCPP_WARN(
        rclcpp::get_logger("fast_lio"),
        "LiDAR timestamp moved backwards. Clearing LiDAR buffers.");

      lidar_buffer.clear();
      time_buffer.clear();
      preprocess_time_buffer.clear();
      lidar_pushed = false;
      ++queue_diagnostics.lidar_timestamp_resets;
    }

    is_first_lidar = false;
    last_timestamp_lidar = current_timestamp;

    lidar_buffer.push_back(std::move(cloud));
    time_buffer.push_back(current_timestamp);
    preprocess_time_buffer.push_back(preprocess_duration);

    ++queue_diagnostics.lidar_received;
    queue_diagnostics.lidar_high_watermark_total = std::max(queue_diagnostics.lidar_high_watermark_total, lidar_buffer.size());
    queue_diagnostics.lidar_high_watermark_interval = std::max(queue_diagnostics.lidar_high_watermark_interval, lidar_buffer.size());
  }
}

double timediff_lidar_wrt_imu = 0.0;
bool   timediff_set_flg = false;
void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg) {
  const double current_timestamp = get_time_sec(msg->header.stamp);
  const double preprocess_start = omp_get_wtime();
  auto cloud = std::make_shared<PointCloudXYZI>();

  // No mantener bloqueado mtx_buffer durante el preprocesado.
  p_pre->process(msg, cloud);

  const double preprocess_duration = omp_get_wtime() - preprocess_start;

  {
    std::lock_guard<std::mutex> lock(mtx_buffer);
    ++scan_count;

    if (!is_first_lidar && current_timestamp < last_timestamp_lidar) {
      RCLCPP_WARN(
        rclcpp::get_logger("fast_lio"),
        "LiDAR timestamp moved backwards. Clearing LiDAR buffers.");

      lidar_buffer.clear();
      time_buffer.clear();
      preprocess_time_buffer.clear();
      lidar_pushed = false;
    }

    is_first_lidar = false;
    last_timestamp_lidar = current_timestamp;

    if (
      !time_sync_en &&
      !imu_buffer.empty() &&
      std::abs(last_timestamp_imu - last_timestamp_lidar) > 10.0)
    {
      RCLCPP_WARN(
        rclcpp::get_logger("fast_lio"),
        "IMU and LiDAR timestamps differ by more than 10 seconds. "
        "IMU=%.9f LiDAR=%.9f",
        last_timestamp_imu,
        last_timestamp_lidar);
    }

    if (
      time_sync_en &&
      !timediff_set_flg &&
      !imu_buffer.empty() &&
      std::abs(last_timestamp_lidar - last_timestamp_imu) > 1.0)
    {
      timediff_set_flg = true;

      timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;

      RCLCPP_INFO(
        rclcpp::get_logger("fast_lio"),
        "Automatic LiDAR-IMU time difference: %.9f s",
        timediff_lidar_wrt_imu);
    }

    lidar_buffer.push_back(std::move(cloud));
    time_buffer.push_back(current_timestamp);
    preprocess_time_buffer.push_back(preprocess_duration);
    ++queue_diagnostics.lidar_received;
    queue_diagnostics.lidar_high_watermark_total = std::max(queue_diagnostics.lidar_high_watermark_total, lidar_buffer.size());
    queue_diagnostics.lidar_high_watermark_interval = std::max(queue_diagnostics.lidar_high_watermark_interval, lidar_buffer.size());
  }
}

void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in) {
  auto msg = std::make_shared<sensor_msgs::msg::Imu>(*msg_in);

  const double raw_timestamp = get_time_sec(msg_in->header.stamp);

  {
    std::lock_guard<std::mutex> lock(mtx_buffer);
    double corrected_timestamp = raw_timestamp - time_diff_lidar_to_imu;
    if (time_sync_en && std::abs(timediff_lidar_wrt_imu) > 0.1) corrected_timestamp = raw_timestamp + timediff_lidar_wrt_imu;
    
    // Usar get_ros_time(). No construir rclcpp::Time directamente
    // con un double expresado en segundos.
    msg->header.stamp = get_ros_time(corrected_timestamp);

    if (corrected_timestamp < last_timestamp_imu) {
      RCLCPP_WARN(
        rclcpp::get_logger("fast_lio"),
        "IMU timestamp moved backwards. Clearing IMU buffer.");

      imu_buffer.clear();
      ++queue_diagnostics.imu_timestamp_resets;
    }
    last_timestamp_imu = corrected_timestamp;
    imu_buffer.push_back(std::move(msg));
    ++queue_diagnostics.imu_received;
    queue_diagnostics.imu_high_watermark_total = std::max(queue_diagnostics.imu_high_watermark_total, imu_buffer.size());
    queue_diagnostics.imu_high_watermark_interval = std::max(queue_diagnostics.imu_high_watermark_interval, imu_buffer.size());
  }
}

double lidar_mean_scantime = 0.0;
int    scan_num = 0;
bool sync_packages(MeasureGroup & meas) {
  std::lock_guard<std::mutex> lock(mtx_buffer);

  if (lidar_buffer.empty() || imu_buffer.empty()) return false;

  // Seleccionar un scan LiDAR, pero mantenerlo en la cola
  // hasta disponer de toda la IMU necesaria.
  if (!lidar_pushed) {
    meas.lidar = lidar_buffer.front();
    meas.lidar_beg_time = time_buffer.front();

    if (!preprocess_time_buffer.empty())
    {
        current_preprocess_time = preprocess_time_buffer.front();
    }
    else
    {
      current_preprocess_time = 0.0;
    }

    if (meas.lidar->points.size() <= 1)
    {
      lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;

      RCLCPP_WARN(
        rclcpp::get_logger("fast_lio"),
        "Too few points in input cloud.");
    }
    else
    {
      const double measured_scan_duration = meas.lidar->points.back().curvature / 1000.0;

      if (measured_scan_duration < 0.5 * lidar_mean_scantime) {
        lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
      }
      else
      {
        ++scan_num;
        lidar_end_time = meas.lidar_beg_time + measured_scan_duration;
        lidar_mean_scantime += (measured_scan_duration - lidar_mean_scantime) /scan_num;
      }
    }

    meas.lidar_end_time = lidar_end_time;
    lidar_pushed = true;
  }

  // Todavía no ha llegado toda la IMU necesaria.
  if (last_timestamp_imu < lidar_end_time)
  {
    ++queue_diagnostics.wait_for_imu_polls_total;
    ++queue_diagnostics.wait_for_imu_polls_interval;

    return false;
  }
  meas.imu.clear();

  while (!imu_buffer.empty())
  {
    const double imu_timestamp = get_time_sec(imu_buffer.front()->header.stamp);

    if (imu_timestamp > lidar_end_time) break;
    

    meas.imu.push_back(imu_buffer.front());
    imu_buffer.pop_front();
  }

  lidar_buffer.pop_front();
  time_buffer.pop_front();

  if (!preprocess_time_buffer.empty()) preprocess_time_buffer.pop_front();
  
  ++queue_diagnostics.synced_scans;
  lidar_pushed = false;
  return true;
}

int process_increments = 0;
void map_incremental() {
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point; 
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false); 
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI());
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull,  const std::string & world_frame, const tf2::Transform & T_map_lio) {
    if (scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();

        PointCloudXYZI::Ptr laserCloudMap(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            PointType point_lio_world;
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], &point_lio_world);

            transform_lio_world_point_to_map(
                point_lio_world,
                &laserCloudMap->points[i],
                T_map_lio);
        }

        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudMap, laserCloudmsg);
        laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
        laserCloudmsg.header.frame_id = world_frame;
        pubLaserCloudFull->publish(laserCloudmsg);
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    /*
    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&feats_undistort->points[i], \
                                &laserCloudWorld->points[i]);
        }
        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
    */
}

void publish_frame_lidar(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher, const std::string & lidar_frame) {
  if (publisher == nullptr || feats_undistort == nullptr || feats_undistort->empty()) return;

  const std::size_t number_of_points = feats_undistort->points.size();

  if (number_of_points > static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())) {
    RCLCPP_ERROR(
      rclcpp::get_logger("fastlio_mapping"),
      "Cannot publish LiDAR cloud: too many points: %zu",
      number_of_points);

    return;
  }

  // La nube Livox es no organizada.
  feats_undistort->width = static_cast<std::uint32_t>(number_of_points);

  feats_undistort->height = 1U;
  feats_undistort->is_dense = false;

  sensor_msgs::msg::PointCloud2 message;

  pcl::toROSMsg(*feats_undistort, message);

  message.header.stamp = get_ros_time(lidar_end_time);

  message.header.frame_id = lidar_frame;

  const std::size_t expected_data_size = static_cast<std::size_t>(message.row_step) * static_cast<std::size_t>(message.height);

  if (
    message.point_step == 0U ||
    message.row_step < message.width * message.point_step ||
    message.data.size() < expected_data_size)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("fastlio_mapping"),
      "Generated invalid PointCloud2: width=%u "
      "height=%u point_step=%u row_step=%u "
      "data_size=%zu expected=%zu",
      message.width,
      message.height,
      message.point_step,
      message.row_step,
      message.data.size(),
      expected_data_size);

    return;
  }

  publisher->publish(message);
}

void publish_effect_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect, const std::string & world_frame, const tf2::Transform & T_map_lio) {
    PointCloudXYZI::Ptr laserCloudMap(new PointCloudXYZI(effct_feat_num, 1));

    for (int i = 0; i < effct_feat_num; i++)
    {
        PointType point_lio_world;
        RGBpointBodyToWorld(&laserCloudOri->points[i], &point_lio_world);

        transform_lio_world_point_to_map(
            point_lio_world,
            &laserCloudMap->points[i],
            T_map_lio);
    }
    sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudMap, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = get_ros_time(lidar_end_time);
    laserCloudFullRes3.header.frame_id = world_frame;
    pubLaserCloudEffect->publish(laserCloudFullRes3);
}

void publish_map(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap, const std::string & world_frame, const tf2::Transform & T_map_lio) {
    PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
    int size = laserCloudFullRes->points.size();

    PointCloudXYZI::Ptr laserCloudMap(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        PointType point_lio_world;
        RGBpointBodyToWorld(&laserCloudFullRes->points[i], &point_lio_world);

        transform_lio_world_point_to_map(
            point_lio_world,
            &laserCloudMap->points[i],
            T_map_lio);
    }

    *pcl_wait_pub += *laserCloudMap;

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*pcl_wait_pub, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = world_frame;
    pubLaserCloudMap->publish(laserCloudmsg);
    // sensor_msgs::msg::PointCloud2 laserCloudMap;
    // pcl::toROSMsg(*featsFromMap, laserCloudMap);
    // laserCloudMap.header.stamp = get_ros_time(lidar_end_time);
    // laserCloudMap.header.frame_id = "camera_init";
    // pubLaserCloudMap->publish(laserCloudMap);
}

void save_to_pcd()
{
    pcl::PCDWriter pcd_writer;
    pcd_writer.writeBinary(map_file_path, *pcl_wait_pub);
}

template<typename T>
void set_posestamp(T & out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
    
}

void publish_odometry(
    const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped,
    std::unique_ptr<tf2_ros::TransformBroadcaster> & tf_br,
    const std::string & world_frame,
    const std::string & body_frame,
    const bool publish_lio_tf,
    const tf2::Transform & T_map_tracking)
{
    odomAftMapped.header.frame_id = world_frame;
    odomAftMapped.child_frame_id = body_frame;
    odomAftMapped.header.stamp = get_ros_time(lidar_end_time);

    set_pose_from_transform(odomAftMapped.pose.pose, T_map_tracking);

    auto P = kf.get_P();
    for (int i = 0; i < 6; i++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i * 6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i * 6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i * 6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i * 6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i * 6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i * 6 + 5] = P(k, 2);
    }

    pubOdomAftMapped->publish(odomAftMapped);

    if (publish_lio_tf)
    {
        geometry_msgs::msg::TransformStamped trans;
        trans.header.frame_id = world_frame;
        trans.header.stamp = odomAftMapped.header.stamp;
        trans.child_frame_id = body_frame;
        trans.transform.translation.x = T_map_tracking.getOrigin().x();
        trans.transform.translation.y = T_map_tracking.getOrigin().y();
        trans.transform.translation.z = T_map_tracking.getOrigin().z();
        trans.transform.rotation = tf2::toMsg(T_map_tracking.getRotation());
        tf_br->sendTransform(trans);
    }
}

void publish_path(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath, const std::string & world_frame, const tf2::Transform & T_map_tracking) {
    set_pose_from_transform(msg_body_pose.pose, T_map_tracking);
    msg_body_pose.header.stamp = get_ros_time(lidar_end_time);
    msg_body_pose.header.frame_id = world_frame;

    static int jjj = 0;
    jjj++;

    if (jjj % 10 == 0)
    {
        path.header.stamp = msg_body_pose.header.stamp;
        path.header.frame_id = world_frame;
        path.poses.push_back(msg_body_pose);
        pubPath->publish(path);
    }
}

void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) {
    double match_start = omp_get_wtime();
    total_residual = 0.0; 

    /** closest surface search and residual computation **/
    #ifdef MP_EN
        #pragma omp parallel for schedule(static) num_threads(openmp_threads)
    #endif
    for (int i = 0; i < feats_down_size; ++i)
    {
        PointType &point_body  = feats_down_body->points[i]; 
        PointType &point_world = feats_down_world->points[i]; 

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }

        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }
    
    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num ++;
        }
    }

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        std::cerr << "No Effective Points!" << std::endl;
        // ROS_WARN("No Effective Points! \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num;
    match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();
    
    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
    solve_time += omp_get_wtime() - solve_start_;
}


struct SavedMapVoxelKey
{
    std::int64_t x;
    std::int64_t y;
    std::int64_t z;

    bool operator==(const SavedMapVoxelKey & other) const noexcept
    {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct SavedMapVoxelKeyHash
{
    std::size_t operator()(const SavedMapVoxelKey & key) const noexcept
    {
        std::size_t seed = 0U;
        const auto combine = [&seed](const std::int64_t value) {
            const std::size_t hashed = std::hash<std::int64_t>{}(value);
            seed ^= hashed + 0x9e3779b97f4a7c15ULL + (seed << 6U) + (seed >> 2U);
        };

        combine(key.x);
        combine(key.y);
        combine(key.z);
        return seed;
    }
};

struct SavedMapVoxelAccumulator
{
    double x_sum = 0.0;
    double y_sum = 0.0;
    double z_sum = 0.0;
    double intensity_sum = 0.0;
    std::uint64_t count = 0U;
};

struct TemporalExportChunk
{
    std::uint64_t chunk_index = 0U;
    std::uint64_t first_scan_id = 0U;
    std::uint64_t last_scan_id = 0U;
    std::uint64_t scan_count = 0U;
    std::vector<TemporalObservationPoint> points;
    std::vector<std::string> scan_rows;
    std::vector<std::string> trajectory_rows;
};

class LaserMappingNode : public rclcpp::Node {
public:
    LaserMappingNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("laser_mapping", options) {
        this->declare_parameter<bool>("publish.path_en", true);
        this->declare_parameter<bool>("publish.effect_map_en", false);
        this->declare_parameter<bool>("publish.map_en", false);
        this->declare_parameter<bool>("publish.scan_publish_en", true);
        this->declare_parameter<bool>("publish.dense_publish_en", true);
        this->declare_parameter<bool>( "publish.scan_worldframe_pub_en", false);
        this->declare_parameter<bool>("publish.scan_lidarframe_pub_en", true);
        this->declare_parameter<int>("max_iteration", 4);
        this->declare_parameter<string>("map_file_path", "");
        this->declare_parameter<string>("common.lid_topic", "/livox/lidar");
        this->declare_parameter<string>("common.imu_topic", "/livox/imu");
        this->declare_parameter<bool>("common.time_sync_en", false);
        this->declare_parameter<double>("common.time_offset_lidar_to_imu", 0.0);
        this->declare_parameter<double>("filter_size_corner", 0.5);
        this->declare_parameter<double>("filter_size_surf", 0.5);
        this->declare_parameter<double>("filter_size_map", 0.5);
        this->declare_parameter<double>("cube_side_length", 500.);
        this->declare_parameter<float>("mapping.det_range", 100.);
        this->declare_parameter<double>("mapping.fov_degree", 180.);
        this->declare_parameter<double>("mapping.gyr_cov", 0.1);
        this->declare_parameter<double>("mapping.acc_cov", 0.1);
        this->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
        this->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
        this->declare_parameter<double>("preprocess.blind", 0.01);
        this->declare_parameter<int>("preprocess.lidar_type", AVIA);
        this->declare_parameter<int>("preprocess.scan_line", 16);
        this->declare_parameter<int>("preprocess.timestamp_unit", US);
        this->declare_parameter<int>("preprocess.scan_rate", 10);
        this->declare_parameter<int>("point_filter_num", 2);
        this->declare_parameter<bool>("feature_extract_enable", false);
        this->declare_parameter<bool>("runtime_pos_log_enable", false);
        this->declare_parameter<bool>("mapping.extrinsic_est_en", true);
        this->declare_parameter<bool>("pcd_save.pcd_save_en", false);
        this->declare_parameter<int>("pcd_save.interval", -1);
        this->declare_parameter<bool>("pcd_save.save_on_shutdown", true);
        this->declare_parameter<double>("pcd_save.voxel_size", 0.15);
        this->declare_parameter<int>("pcd_save.scan_stride", 1);
        this->declare_parameter<bool>("pcd_save.use_dense_cloud", true);
        this->declare_parameter<std::int64_t>("pcd_save.max_voxels", 20000000);
        this->declare_parameter<std::int64_t>("pcd_save.reserve_voxels", 1000000);
        this->declare_parameter<double>("pcd_save.min_range", 0.5);
        this->declare_parameter<double>("pcd_save.max_range", 80.0);
        this->declare_parameter<bool>("pcd_save.temporal_export.enabled", false);
        this->declare_parameter<std::string>("pcd_save.temporal_export.output_dir", "");
        this->declare_parameter<int>("pcd_save.temporal_export.scans_per_chunk", 200);
        this->declare_parameter<int>("pcd_save.temporal_export.max_pending_chunks", 2);
        this->declare_parameter<int>("pcd_save.temporal_export.scan_stride", 1);
        this->declare_parameter<int>("pcd_save.temporal_export.point_stride", 1);
        this->declare_parameter<bool>("pcd_save.temporal_export.use_dense_cloud", true);
        this->declare_parameter<bool>("pcd_save.temporal_export.write_trajectory_knots", true);
        this->declare_parameter<std::string>("frames.lio_world_frame", "map");
        this->declare_parameter<std::string>("frames.lio_body_frame", "body");
        this->declare_parameter<bool>("rep105.enable", false);
        this->declare_parameter<bool>("rep105.publish_map_to_odom_tf", false);
        this->declare_parameter<bool>("rep105.publish_lio_tf", true);
        this->declare_parameter<std::string>("rep105.map_frame", "map");
        this->declare_parameter<std::string>("rep105.odom_frame", "odom");
        this->declare_parameter<std::string>("rep105.robot_tracking_frame", "mid360");
        this->declare_parameter<bool>("rep105.use_latest_robot_tf", true);
        this->declare_parameter<double>("rep105.tf_timeout_sec", 0.05);
        this->declare_parameter<bool>("rep105.align_map_to_odom_on_start", true);
        this->declare_parameter<bool>("rep105.project_map_to_2d", false);
        this->declare_parameter<vector<double>>("mapping.extrinsic_T", vector<double>());
        this->declare_parameter<vector<double>>("mapping.extrinsic_R", vector<double>());
    
        this->declare_parameter<int>("processing.openmp_threads", 2);
        this->declare_parameter<int>("processing.executor_threads", 3);
        this->declare_parameter<int>("processing.lidar_qos_depth", 50);
        this->declare_parameter<int>("processing.imu_qos_depth", 400);
        this->declare_parameter<bool>("processing.deskew_only", false);
        this->declare_parameter<bool>("processing.queue_stats_enabled", true);
        this->declare_parameter<double>("processing.queue_stats_period_sec", 1.0);
        this->declare_parameter<int>("processing.lidar_queue_warn_size", 3);
        this->declare_parameter<int>("processing.imu_queue_warn_size", 100);
        this->declare_parameter<std::string>("publish.deskewed_topic", "/cloud_registered_lidar");


        this->get_parameter_or<bool>("publish.path_en", path_en, true);
        this->get_parameter_or<bool>("publish.effect_map_en", effect_pub_en, false);
        this->get_parameter_or<bool>("publish.map_en", map_pub_en, false);
        this->get_parameter_or<bool>("publish.scan_publish_en", scan_pub_en, true);
        this->get_parameter_or<bool>("publish.dense_publish_en", dense_pub_en, true);
        this->get_parameter_or<bool>("publish.scan_worldframe_pub_en", scan_world_pub_en, false);
        this->get_parameter_or<bool>("publish.scan_lidarframe_pub_en", scan_lidar_pub_en, true);
        this->get_parameter_or<int>("max_iteration", NUM_MAX_ITERATIONS, 4);
        this->get_parameter_or<string>("map_file_path", map_file_path, "");
        this->get_parameter_or<string>("common.lid_topic", lid_topic, "/livox/lidar");
        this->get_parameter_or<string>("common.imu_topic", imu_topic,"/livox/imu");
        this->get_parameter_or<bool>("common.time_sync_en", time_sync_en, false);
        this->get_parameter_or<double>("common.time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
        this->get_parameter_or<double>("filter_size_corner",filter_size_corner_min,0.5);
        this->get_parameter_or<double>("filter_size_surf",filter_size_surf_min,0.5);
        this->get_parameter_or<double>("filter_size_map",filter_size_map_min,0.5);
        this->get_parameter_or<double>("cube_side_length",cube_len,500.f);
        this->get_parameter_or<float>("mapping.det_range",DET_RANGE,100.f);
        this->get_parameter_or<double>("mapping.fov_degree",fov_deg,180.f);
        this->get_parameter_or<double>("mapping.gyr_cov",gyr_cov,0.1);
        this->get_parameter_or<double>("mapping.acc_cov",acc_cov,0.1);
        this->get_parameter_or<double>("mapping.b_gyr_cov",b_gyr_cov,0.0001);
        this->get_parameter_or<double>("mapping.b_acc_cov",b_acc_cov,0.0001);
        this->get_parameter_or<double>("preprocess.blind", p_pre->blind, 0.01);
        this->get_parameter_or<int>("preprocess.lidar_type", p_pre->lidar_type, AVIA);
        this->get_parameter_or<int>("preprocess.scan_line", p_pre->N_SCANS, 16);
        this->get_parameter_or<int>("preprocess.timestamp_unit", p_pre->time_unit, US);
        this->get_parameter_or<int>("preprocess.scan_rate", p_pre->SCAN_RATE, 10);
        this->get_parameter_or<int>("point_filter_num", p_pre->point_filter_num, 2);
        this->get_parameter_or<bool>("feature_extract_enable", p_pre->feature_enabled, false);
        this->get_parameter_or<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
        this->get_parameter_or<bool>("mapping.extrinsic_est_en", extrinsic_est_en, true);
        this->get_parameter_or<bool>("pcd_save.pcd_save_en", pcd_save_en, false);
        this->get_parameter_or<int>("pcd_save.interval", pcd_save_interval, -1);
        this->get_parameter_or<bool>("pcd_save.save_on_shutdown", pcd_save_on_shutdown_, true);
        this->get_parameter_or<double>("pcd_save.voxel_size", pcd_save_voxel_size_, 0.15);
        this->get_parameter_or<int>("pcd_save.scan_stride", pcd_save_scan_stride_, 1);
        this->get_parameter_or<bool>("pcd_save.use_dense_cloud", pcd_save_use_dense_cloud_, true);
        this->get_parameter_or<std::int64_t>("pcd_save.max_voxels", pcd_save_max_voxels_, 20000000);
        this->get_parameter_or<std::int64_t>("pcd_save.reserve_voxels", pcd_save_reserve_voxels_, 1000000);
        this->get_parameter_or<double>("pcd_save.min_range", pcd_save_min_range_, 0.5);
        this->get_parameter_or<double>("pcd_save.max_range", pcd_save_max_range_, 80.0);
        this->get_parameter_or<bool>("pcd_save.temporal_export.enabled", temporal_export_enabled_, false);
        this->get_parameter_or<std::string>("pcd_save.temporal_export.output_dir", temporal_output_dir_, "");
        this->get_parameter_or<int>("pcd_save.temporal_export.scans_per_chunk", temporal_scans_per_chunk_, 200);
        this->get_parameter_or<int>("pcd_save.temporal_export.max_pending_chunks", temporal_max_pending_chunks_, 2);
        this->get_parameter_or<int>("pcd_save.temporal_export.scan_stride", temporal_scan_stride_, 1);
        this->get_parameter_or<int>("pcd_save.temporal_export.point_stride", temporal_point_stride_, 1);
        this->get_parameter_or<bool>("pcd_save.temporal_export.use_dense_cloud", temporal_use_dense_cloud_, true);
        this->get_parameter_or<bool>("pcd_save.temporal_export.write_trajectory_knots", temporal_write_trajectory_knots_, true);
        this->get_parameter_or<std::string>("frames.lio_world_frame", lio_world_frame_, "map");
        this->get_parameter_or<std::string>("frames.lio_body_frame", lio_body_frame_, "body");
        this->get_parameter_or<bool>("rep105.enable", rep105_enable_, false);
        this->get_parameter_or<bool>("rep105.publish_map_to_odom_tf", rep105_publish_map_to_odom_tf_, false);
        this->get_parameter_or<bool>("rep105.publish_lio_tf", rep105_publish_lio_tf_, true);
        this->get_parameter_or<std::string>("rep105.map_frame", rep105_map_frame_, "map");
        this->get_parameter_or<std::string>("rep105.odom_frame", rep105_odom_frame_, "odom");
        this->get_parameter_or<std::string>("rep105.robot_tracking_frame", rep105_robot_tracking_frame_, "mid360");
        this->get_parameter_or<bool>("rep105.use_latest_robot_tf", rep105_use_latest_robot_tf_, true);
        this->get_parameter_or<double>("rep105.tf_timeout_sec", rep105_tf_timeout_sec_, 0.05);
        this->get_parameter_or<bool>("rep105.align_map_to_odom_on_start", rep105_align_map_to_odom_on_start_, true);
        this->get_parameter_or<bool>("rep105.project_map_to_2d", rep105_project_map_to_2d_, false);
        this->get_parameter_or<vector<double>>("mapping.extrinsic_T", extrinT, vector<double>());
        this->get_parameter_or<vector<double>>("mapping.extrinsic_R", extrinR, vector<double>());

        this->get_parameter_or<int>("processing.openmp_threads", openmp_threads, 2);
        this->get_parameter_or<int>("processing.executor_threads", executor_threads_, 3);
        this->get_parameter_or<int>("processing.lidar_qos_depth", lidar_qos_depth_, 50);
        this->get_parameter_or<int>("processing.imu_qos_depth", imu_qos_depth_, 400);
        this->get_parameter_or<bool>("processing.deskew_only", deskew_only_, false);
        this->get_parameter_or<bool>("processing.queue_stats_enabled", queue_stats_enabled_, true);
        this->get_parameter_or<double>("processing.queue_stats_period_sec", queue_stats_period_sec_, 1.0);
        this->get_parameter_or<int>("processing.lidar_queue_warn_size", lidar_queue_warn_size_, 3);
        this->get_parameter_or<int>("processing.imu_queue_warn_size", imu_queue_warn_size_, 100);
        this->get_parameter_or<std::string>("publish.deskewed_topic", deskewed_topic_, "/cloud_registered_lidar");

        if (deskew_only_)
        {
            // Keep synchronization, IMU initialization/prediction and point
            // motion compensation, but disable the LiDAR scan-to-map backend.
            path_en = false;
            effect_pub_en = false;
            map_pub_en = false;
            scan_world_pub_en = false;
            scan_lidar_pub_en = true;
            rep105_enable_ = false;
            rep105_publish_map_to_odom_tf_ = false;
            rep105_publish_lio_tf_ = false;
            pcd_save_en = false;
            temporal_export_enabled_ = false;

            RCLCPP_WARN(
                this->get_logger(),
                "FAST-LIO deskew-only mode enabled. Publishing only '%s' in frame '%s'. "
                "LiDAR scan-to-map updates, map construction, odometry, path and TF are disabled. "
                "The deskew trajectory is IMU-predicted and is not corrected by LiDAR registration.",
                deskewed_topic_.c_str(),
                lio_body_frame_.c_str());
        }

        

        // Validaciones.
        if (openmp_threads < 1)
        {
            throw std::invalid_argument("processing.openmp_threads must be >= 1");
        }

        if (executor_threads_ < 2)
        {
            throw std::invalid_argument("processing.executor_threads must be >= 2");
        }

        if (lidar_qos_depth_ < 1)
        {
            throw std::invalid_argument("processing.lidar_qos_depth must be >= 1");
        }

        if (imu_qos_depth_ < 1)
        {
            throw std::invalid_argument("processing.imu_qos_depth must be >= 1");
        }

        if (filter_size_surf_min <= 0.0 || filter_size_map_min <= 0.0)
        {
            throw std::invalid_argument("filter_size_surf and filter_size_map must be > 0");
        }

        if (pcd_save_en)
        {
            if (map_file_path.empty())
            {
                throw std::invalid_argument(
                    "map_file_path must not be empty when pcd_save.pcd_save_en is true");
            }

            if (pcd_save_voxel_size_ <= 0.0)
            {
                throw std::invalid_argument("pcd_save.voxel_size must be > 0");
            }

            if (pcd_save_scan_stride_ < 1)
            {
                throw std::invalid_argument("pcd_save.scan_stride must be >= 1");
            }

            if (pcd_save_max_voxels_ < 0 || pcd_save_reserve_voxels_ < 0)
            {
                throw std::invalid_argument(
                    "pcd_save.max_voxels and pcd_save.reserve_voxels must be >= 0");
            }

            if (
                pcd_save_min_range_ < 0.0 ||
                pcd_save_max_range_ <= pcd_save_min_range_)
            {
                throw std::invalid_argument(
                    "pcd_save range must satisfy 0 <= min_range < max_range");
            }

            if (pcd_save_reserve_voxels_ > 0)
            {
                saved_map_voxels_.reserve(
                    static_cast<std::size_t>(pcd_save_reserve_voxels_));
            }
        }

        if (temporal_export_enabled_)
        {
            if (temporal_output_dir_.empty() && map_file_path.empty())
            {
                throw std::invalid_argument(
                    "pcd_save.temporal_export.output_dir must be set when "
                    "map_file_path is empty");
            }

            if (temporal_scans_per_chunk_ < 1)
            {
                throw std::invalid_argument(
                    "pcd_save.temporal_export.scans_per_chunk must be >= 1");
            }

            if (temporal_max_pending_chunks_ < 1)
            {
                throw std::invalid_argument(
                    "pcd_save.temporal_export.max_pending_chunks must be >= 1");
            }

            if (temporal_scan_stride_ < 1 || temporal_point_stride_ < 1)
            {
                throw std::invalid_argument(
                    "pcd_save.temporal_export scan_stride and point_stride must be >= 1");
            }

            if (
                pcd_save_min_range_ < 0.0 ||
                pcd_save_max_range_ <= pcd_save_min_range_)
            {
                throw std::invalid_argument(
                    "pcd_save range must satisfy 0 <= min_range < max_range");
            }

        }

        const double minimum_cube_side = 2.0 * MOV_THRESHOLD * static_cast<double>(DET_RANGE);
        if (cube_len <= minimum_cube_side)
        {
            throw std::invalid_argument(
                "cube_side_length must be greater than "
                "3 * mapping.det_range");
        }
        if (queue_stats_period_sec_ <= 0.0)
        {
            throw std::invalid_argument("processing.queue_stats_period_sec must be > 0");
        }

        if (lidar_queue_warn_size_ < 1)
        {
            throw std::invalid_argument("processing.lidar_queue_warn_size must be >= 1");
        }

        if (imu_queue_warn_size_ < 1)
        {
            throw std::invalid_argument("processing.imu_queue_warn_size must be >= 1");
        }

        if (temporal_export_enabled_)
        {
            initialize_temporal_export();
        }

        RCLCPP_INFO(this->get_logger(), "p_pre->lidar_type %d", p_pre->lidar_type);
        // Logs de la configuración efectiva.
        RCLCPP_INFO(
            this->get_logger(),
            "Processing configuration: "
            "openmp_threads=%d, "
            "executor_threads=%d, "
            "lidar_qos_depth=%d, "
            "imu_qos_depth=%d",
            openmp_threads,
            executor_threads_,
            lidar_qos_depth_,
            imu_qos_depth_);
        RCLCPP_INFO(
            this->get_logger(),
            "Queue diagnostics: enabled=%s, period=%.2f s, "
            "lidar_warn_size=%d, imu_warn_size=%d",
            queue_stats_enabled_ ? "true" : "false",
            queue_stats_period_sec_,
            lidar_queue_warn_size_,
            imu_queue_warn_size_);

        RCLCPP_INFO(
            this->get_logger(),
            "Map saving: enabled=%s, path='%s', save_on_shutdown=%s, "
            "voxel_size=%.3f m, scan_stride=%d, dense=%s, "
            "range=[%.1f, %.1f] m, max_voxels=%ld",
            pcd_save_en ? "true" : "false",
            map_file_path.c_str(),
            pcd_save_on_shutdown_ ? "true" : "false",
            pcd_save_voxel_size_,
            pcd_save_scan_stride_,
            pcd_save_use_dense_cloud_ ? "true" : "false",
            pcd_save_min_range_,
            pcd_save_max_range_,
            static_cast<long>(pcd_save_max_voxels_));

        RCLCPP_INFO(
            this->get_logger(),
            "Temporal export: enabled=%s, output_dir='%s', scans_per_chunk=%d, "
            "max_pending_chunks=%d, scan_stride=%d, point_stride=%d, dense=%s, "
            "trajectory_knots=%s",
            temporal_export_enabled_ ? "true" : "false",
            temporal_output_dir_.c_str(),
            temporal_scans_per_chunk_,
            temporal_max_pending_chunks_,
            temporal_scan_stride_,
            temporal_point_stride_,
            temporal_use_dense_cloud_ ? "true" : "false",
            temporal_write_trajectory_knots_ ? "true" : "false");

        #ifdef MP_EN
        RCLCPP_INFO(
        this->get_logger(),
        "OpenMP enabled: configured_threads=%d, available_cpu_threads=%d",
        openmp_threads,
        omp_get_num_procs());
        #else
        RCLCPP_WARN(
        this->get_logger(),
        "OpenMP is disabled at compile time. "
        "Point-to-map correspondence search will be sequential.");
        #endif

        path.header.stamp = this->get_clock()->now();
        path.header.frame_id = lio_world_frame_;
        T_map_lio_initial_.setIdentity();
        latest_T_map_lio_.setIdentity();

        // /*** variables definition ***/
        // int effect_feat_num = 0, frame_num = 0;
        // double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
        // bool flg_EKF_converged, EKF_stop_flg = 0;

        FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
        HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

        _featsArray.reset(new PointCloudXYZI());

        downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
        downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);

        Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
        Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
        p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
        p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
        p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
        p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
        p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

        fill(epsi, epsi+23, 0.001);
        kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

        /*** debug record ***/
        // FILE *fp;
        if (runtime_pos_log)
        {
            const std::string pos_log_dir = root_dir + "/Log/pos_log.txt";
            fp = fopen(pos_log_dir.c_str(), "w");

            fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), std::ios::out);
            fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), std::ios::out);
            fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"), std::ios::out);

            if (fp == nullptr || !fout_pre.is_open() || !fout_out.is_open())
            {
                throw std::runtime_error( "Failed to open FAST-LIO runtime log files");
            }

            RCLCPP_INFO(this->get_logger(),"Runtime position logging enabled.");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(),"Runtime position logging disabled.");
        }

        // Los tres grupos son secuenciales internamente, pero pueden
        // ejecutarse concurrentemente entre sí.
        lidar_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        imu_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        mapping_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions lidar_options;
        lidar_options.callback_group = lidar_callback_group_;
        rclcpp::SubscriptionOptions imu_options;
        imu_options.callback_group = imu_callback_group_;

        /*** ROS subscribe initialization ***/
        if (p_pre->lidar_type == AVIA)
        {
            auto lidar_qos =rclcpp::QoS(rclcpp::KeepLast(lidar_qos_depth_));
            lidar_qos.reliable().durability_volatile();
            sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, lidar_qos, livox_pcl_cbk, lidar_options);
        }
        else
        {
            sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, rclcpp::SensorDataQoS(), standard_pcl_cbk, lidar_options);
        }

        auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(imu_qos_depth_));
        imu_qos.reliable().durability_volatile();
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, imu_qos, imu_cbk, imu_options);

        pubLaserCloudFull_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 20);
        pubLaserCloudFull_lidar_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(deskewed_topic_, rclcpp::SensorDataQoS());
        pubLaserCloudEffect_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 20);
        pubLaserCloudMap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 20);
        pubOdomAftMapped_ = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 20);
        pubPath_ = this->create_publisher<nav_msgs::msg::Path>("/path", 20);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        //------------------------------------------------------------------------------------------------------
        const auto mapping_period = std::chrono::milliseconds(10);
        timer_ = this->create_wall_timer(mapping_period, std::bind(&LaserMappingNode::timer_callback, this), mapping_callback_group_);
        if (!deskew_only_)
        {
            const auto map_publish_period = std::chrono::seconds(1);
            map_pub_timer_ = this->create_wall_timer(
                map_publish_period,
                std::bind(&LaserMappingNode::map_publish_callback, this),
                mapping_callback_group_);
        }
        if (queue_stats_enabled_)
        {
            const auto queue_stats_period = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(queue_stats_period_sec_));
            previous_queue_stats_time_ = std::chrono::steady_clock::now();
            queue_stats_timer_ = this->create_wall_timer(queue_stats_period, std::bind(&LaserMappingNode::queue_stats_callback, this), mapping_callback_group_);
        }
        if (!deskew_only_)
        {
            map_save_srv_ = this->create_service<std_srvs::srv::Trigger>(
                "map_save",
                std::bind(
                    &LaserMappingNode::map_save_callback,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2),
                rclcpp::ServicesQoS(),
                mapping_callback_group_);
        }

        if (temporal_export_enabled_)
        {
            temporal_writer_thread_ = std::thread(
                &LaserMappingNode::temporal_writer_loop,
                this);
        }

        RCLCPP_INFO(this->get_logger(), "Node init finished.");
    }

    ~LaserMappingNode() override {
        if (temporal_export_enabled_)
        {
            std::string temporal_message;
            if (!shutdown_temporal_export(temporal_message))
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Temporal export shutdown failed: %s",
                    temporal_message.c_str());
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "%s", temporal_message.c_str());
            }
        }

        if (pcd_save_en && pcd_save_on_shutdown_ && saved_map_dirty_)
        {
            std::string save_message;
            if (!save_accumulated_map(save_message))
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Automatic map save failed during shutdown: %s",
                    save_message.c_str());
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "%s", save_message.c_str());
            }
        }

        if (fout_out.is_open()) fout_out.close();
        if (fout_pre.is_open()) fout_pre.close();
        if (fout_dbg.is_open()) fout_dbg.close();

        if (fp != nullptr)
        {
            fclose(fp);
            fp = nullptr;
        }
    }

    std::size_t executor_thread_count() const noexcept {
        return static_cast<std::size_t>(executor_threads_);
    }

private:

    bool compute_map_alignment(tf2::Transform & T_map_lio, tf2::Transform & T_map_tracking, tf2::Transform & T_odom_tracking) {
        const tf2::Transform T_lio_tracking = make_lio_tracking_transform();

        if (!rep105_enable_ || !rep105_publish_map_to_odom_tf_)
        {
            T_map_lio.setIdentity();
            T_map_tracking = T_lio_tracking;
            T_odom_tracking.setIdentity();

            latest_T_map_lio_ = T_map_lio;
            latest_map_alignment_ready_ = true;

            return true;
        }

        geometry_msgs::msg::TransformStamped odom_to_tracking_msg;

        try {
            if (rep105_use_latest_robot_tf_) {
                odom_to_tracking_msg = tf_buffer_->lookupTransform(
                    rep105_odom_frame_,
                    rep105_robot_tracking_frame_,
                    tf2::TimePointZero,
                    tf2::durationFromSec(rep105_tf_timeout_sec_));
            } else {
                odom_to_tracking_msg = tf_buffer_->lookupTransform(
                    rep105_odom_frame_,
                    rep105_robot_tracking_frame_,
                    get_ros_time(lidar_end_time),
                    tf2::durationFromSec(rep105_tf_timeout_sec_));
            }
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Cannot lookup TF %s -> %s: %s",
                rep105_odom_frame_.c_str(),
                rep105_robot_tracking_frame_.c_str(),
                ex.what());
            return false;
        }

        tf2::fromMsg(odom_to_tracking_msg.transform, T_odom_tracking);

        if (!rep105_initial_alignment_ready_)
        {
            if (rep105_align_map_to_odom_on_start_)
            {
                T_map_lio_initial_ =
                    T_odom_tracking * T_lio_tracking.inverse();
            }
            else
            {
                T_map_lio_initial_.setIdentity();
            }

            rep105_initial_alignment_ready_ = true;

            RCLCPP_INFO(
                this->get_logger(),
                "REP-105 initial alignment captured. align_map_to_odom_on_start=%s",
                rep105_align_map_to_odom_on_start_ ? "true" : "false");
        }

        const tf2::Transform T_map_tracking_full =
            T_map_lio_initial_ * T_lio_tracking;

        tf2::Transform T_map_odom =
            T_map_tracking_full * T_odom_tracking.inverse();

        if (rep105_project_map_to_2d_)
        {
            T_map_odom = project_transform_to_se2(T_map_odom);

            // Recompute the tracking pose so all published global outputs are
            // consistent with the projected map -> odom TF.
            T_map_tracking = T_map_odom * T_odom_tracking;
            T_map_lio = T_map_tracking * T_lio_tracking.inverse();
        }
        else
        {
            T_map_tracking = T_map_tracking_full;
            T_map_lio = T_map_lio_initial_;
        }

        latest_T_map_lio_ = T_map_lio;
        latest_map_alignment_ready_ = true;

        return true;
    }

    void timer_callback() {
        if(sync_packages(Measures))
        {
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                return;
            }

            double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time   = 0;
            t0 = omp_get_wtime();

            p_imu->Process(Measures, kf, feats_undistort);
            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

            if (!feats_undistort || feats_undistort->empty()) {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
                return;
            }

            if (deskew_only_)
            {
                publish_frame_lidar(pubLaserCloudFull_lidar_, lio_body_frame_);
                mapping_completed_count.fetch_add(1, std::memory_order_relaxed);
                return;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;
            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();

            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            t1 = omp_get_wtime();
            feats_down_size = feats_down_body->points.size();
            /*** initialize the map kdtree ***/
            if(ikdtree.Root_Node == nullptr)
            {
                RCLCPP_INFO(this->get_logger(), "Initialize the map kdtree");
                if(feats_down_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for(int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    ikdtree.Build(feats_down_world->points);
                }
                return;
            }
            int featsFromMapNum = ikdtree.validnum();
            kdtree_size_st = ikdtree.size();
            
            // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5)
            {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
                return;
            }
            
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            if (runtime_pos_log)
            {
                const V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);

                fout_pre
                    << std::setw(20)
                    << Measures.lidar_beg_time - first_lidar_time
                    << ' '
                    << euler_cur.transpose()
                    << ' '
                    << state_point.pos.transpose()
                    << ' '
                    << ext_euler.transpose()
                    << ' '
                    << state_point.offset_T_L_I.transpose()
                    << ' '
                    << state_point.vel.transpose()
                    << ' '
                    << state_point.bg.transpose()
                    << ' '
                    << state_point.ba.transpose()
                    << ' '
                    << state_point.grav
                    << '\n';
            }

            if(0) // If you need to see map point, change to "if(1)"
            {
                PointVector ().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
            }

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            int  rematch_num = 0;
            bool nearest_search_en = true; //

            t2 = omp_get_wtime();
            
            /*** iterated state estimation ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            const std::size_t feature_count = static_cast<std::size_t>(feats_down_size);
            normvec->resize(feature_count);
            feats_down_world->resize(feature_count);
            laserCloudOri->resize(feature_count);
            corr_normvect->resize(feature_count);
            res_last.assign(feature_count,0.0F);
            point_selected_surf.assign(feature_count,1U);
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
            state_point = kf.get_x();
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];

            double t_update_end = omp_get_wtime();

            /******* Compute REP-105 map alignment *******/
            tf2::Transform T_map_lio;
            tf2::Transform T_map_tracking;
            tf2::Transform T_odom_tracking;

            if (!compute_map_alignment(T_map_lio, T_map_tracking, T_odom_tracking)) return;

            /******* Publish odometry in real map frame *******/
            publish_odometry(pubOdomAftMapped_, tf_broadcaster_, lio_world_frame_, lio_body_frame_, rep105_publish_lio_tf_, T_map_tracking);

            /******* Publish REP-105 map -> odom *******/
            publish_map_to_odom_tf(T_map_tracking, T_odom_tracking);

            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();
            map_incremental();
            t5 = omp_get_wtime();

            /******* Accumulate a bounded-density global map for PCD/PLY export *******/
            accumulate_map_for_save(T_map_lio);

            /******* Publish points *******/
            if (path_en)  publish_path(pubPath_, lio_world_frame_, T_map_tracking);
            if (scan_pub_en && scan_world_pub_en) publish_frame_world(pubLaserCloudFull_, lio_world_frame_, T_map_lio);
            if (scan_pub_en && scan_lidar_pub_en) publish_frame_lidar(pubLaserCloudFull_lidar_, lio_body_frame_);
            if (effect_pub_en) publish_effect_world(pubLaserCloudEffect_, lio_world_frame_, T_map_lio);

            // if (map_pub_en) publish_map(pubLaserCloudMap_, lio_world_frame_, T_map_lio);

            mapping_completed_count.fetch_add(1, std::memory_order_relaxed);

            /*** Debug variables ***/
            if (runtime_pos_log)
            {
                frame_num ++;
                kdtree_size_end = ikdtree.size();
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + (t_update_end - t_update_start) / frame_num;
                aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;
                aver_time_incre = aver_time_incre * (frame_num - 1)/frame_num + (kdtree_incremental_time)/frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + (solve_time + solve_H_time)/frame_num;
                aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1)/frame_num + solve_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = kdtree_incremental_time;
                s_plot4[time_log_counter] = kdtree_search_time;
                s_plot5[time_log_counter] = kdtree_delete_counter;
                s_plot6[time_log_counter] = kdtree_delete_time;
                s_plot7[time_log_counter] = kdtree_size_st;
                s_plot8[time_log_counter] = kdtree_size_end;
                s_plot9[time_log_counter] = aver_time_consu;
                s_plot10[time_log_counter] = add_point_size;
                s_plot11[time_log_counter] = current_preprocess_time;
                const V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
                fout_out
                    << std::setw(20)
                    << Measures.lidar_beg_time - first_lidar_time
                    << ' '
                    << euler_cur.transpose()
                    << ' '
                    << state_point.pos.transpose()
                    << ' '
                    << ext_euler.transpose()
                    << ' '
                    << state_point.offset_T_L_I.transpose()
                    << ' '
                    << state_point.vel.transpose()
                    << ' '
                    << state_point.bg.transpose()
                    << ' '
                    << state_point.ba.transpose()
                    << ' '
                    << state_point.grav
                    << ' '
                    << feats_undistort->points.size()
                    << '\n';
                dump_lio_state_to_log(fp);
                ++time_log_counter;
            }
        }
    }

    void map_publish_callback() {
        if (!map_pub_en) return;
        if (rep105_enable_ && rep105_publish_map_to_odom_tf_ && !latest_map_alignment_ready_) return;

        publish_map(pubLaserCloudMap_, lio_world_frame_, latest_T_map_lio_);
    }

    std::string temporal_global_frame_id() const
    {
        if (rep105_enable_ && rep105_publish_map_to_odom_tf_)
        {
            return rep105_map_frame_;
        }
        return lio_world_frame_;
    }

    void initialize_temporal_export()
    {
        namespace fs = std::filesystem;

        fs::path requested_path;
        if (!temporal_output_dir_.empty())
        {
            requested_path = fs::path(temporal_output_dir_);
        }
        else
        {
            const fs::path map_path(map_file_path);
            requested_path = map_path.parent_path() /
                (map_path.stem().string() + "_temporal");
        }

        fs::path selected_path = requested_path;
        for (std::uint32_t suffix = 1U;; ++suffix)
        {
            std::error_code filesystem_error;
            const bool path_exists = fs::exists(selected_path, filesystem_error);
            if (filesystem_error)
            {
                throw std::runtime_error(
                    "Cannot inspect temporal export path '" +
                    selected_path.string() + "': " + filesystem_error.message());
            }

            if (!path_exists)
            {
                break;
            }

            const bool is_directory = fs::is_directory(selected_path, filesystem_error);
            if (filesystem_error)
            {
                throw std::runtime_error(
                    "Cannot inspect temporal export path type '" +
                    selected_path.string() + "': " + filesystem_error.message());
            }

            if (is_directory)
            {
                const bool is_empty = fs::is_empty(selected_path, filesystem_error);
                if (filesystem_error)
                {
                    throw std::runtime_error(
                        "Cannot inspect temporal export directory '" +
                        selected_path.string() + "': " + filesystem_error.message());
                }
                if (is_empty)
                {
                    break;
                }
            }

            std::ostringstream suffixed_name;
            suffixed_name << requested_path.string() << "_" <<
                std::setw(3) << std::setfill('0') << suffix;
            selected_path = fs::path(suffixed_name.str());
        }

        std::error_code filesystem_error;
        fs::create_directories(selected_path, filesystem_error);
        if (filesystem_error)
        {
            throw std::runtime_error(
                "Cannot create temporal export directory '" +
                selected_path.string() + "': " + filesystem_error.message());
        }

        temporal_output_dir_ = selected_path.string();

        {
            std::ofstream scans_file(selected_path / "scans.csv", std::ios::out | std::ios::trunc);
            if (!scans_file.is_open())
            {
                throw std::runtime_error("Cannot create temporal scans.csv");
            }
            scans_file <<
                "scan_id,begin_time_sec,end_time_sec,source_points,exported_points,"
                "trajectory_knots,start_x,start_y,start_z,start_qx,start_qy,start_qz,start_qw,"
                "end_x,end_y,end_z,end_qx,end_qy,end_qz,end_qw\n";
        }

        {
            std::ofstream trajectory_file(
                selected_path / "trajectory.csv",
                std::ios::out | std::ios::trunc);
            if (!trajectory_file.is_open())
            {
                throw std::runtime_error("Cannot create temporal trajectory.csv");
            }
            trajectory_file <<
                "scan_id,knot_index,time_offset_sec,timestamp_sec,x,y,z,qx,qy,qz,qw\n";
        }

        {
            std::ofstream chunks_file(selected_path / "chunks.csv", std::ios::out | std::ios::trunc);
            if (!chunks_file.is_open())
            {
                throw std::runtime_error("Cannot create temporal chunks.csv");
            }
            chunks_file <<
                "chunk_index,filename,first_scan_id,last_scan_id,scan_count,point_count\n";
        }

        {
            std::ofstream metadata_file(
                selected_path / "metadata.yaml",
                std::ios::out | std::ios::trunc);
            if (!metadata_file.is_open())
            {
                throw std::runtime_error("Cannot create temporal metadata.yaml");
            }

            metadata_file <<
                "format_version: 1\n"
                "frame_id: \"" << temporal_global_frame_id() << "\"\n"
                "point_cloud_format: pcd_binary\n"
                "point_record_size_bytes: 32\n"
                "point_fields:\n"
                "  - {name: x, type: float32, unit: m}\n"
                "  - {name: y, type: float32, unit: m}\n"
                "  - {name: z, type: float32, unit: m}\n"
                "  - {name: intensity, type: float32}\n"
                "  - {name: time_offset_sec, type: float32, unit: s}\n"
                "  - {name: scan_id, type: uint32}\n"
                "  - {name: timestamp_sec, type: float64, unit: s}\n"
                "timestamp_reference: ROS time from the original LiDAR message\n"
                "time_offset_reference: scan begin\n"
                "trajectory_knots_enabled: " <<
                    (temporal_write_trajectory_knots_ ? "true" : "false") << "\n"
                "trajectory_semantics: >-\n"
                "  IMU-predicted intra-scan trajectory rigidly corrected so the final\n"
                "  LiDAR pose matches the post-EKF FAST-LIO state. Interpolate trajectory.csv\n"
                "  by scan_id and time_offset_sec to recover the LiDAR ray origin.\n"
                "range_min_m: " << std::setprecision(17) << pcd_save_min_range_ << "\n"
                "range_max_m: " << pcd_save_max_range_ << "\n"
                "scan_stride: " << temporal_scan_stride_ << "\n"
                "point_stride: " << temporal_point_stride_ << "\n"
                "use_dense_cloud: " << (temporal_use_dense_cloud_ ? "true" : "false") << "\n";
        }
    }

    bool write_temporal_chunk(
        TemporalExportChunk & chunk,
        std::string & error_message)
    {
        namespace fs = std::filesystem;

        std::ostringstream file_name_stream;
        file_name_stream << "observations_" << std::setw(6) << std::setfill('0') <<
            chunk.chunk_index << ".pcd";
        const std::string file_name = file_name_stream.str();

        const fs::path output_path = fs::path(temporal_output_dir_) / file_name;
        fs::path temporary_path = output_path;
        temporary_path += ".tmp";

        const std::uint16_t endian_probe = 1U;
        if (*reinterpret_cast<const std::uint8_t *>(&endian_probe) != 1U)
        {
            error_message =
                "Temporal PCD binary writer currently requires a little-endian host.";
            return false;
        }

        std::ofstream pcd_file(
            temporary_path,
            std::ios::out | std::ios::binary | std::ios::trunc);
        if (!pcd_file.is_open())
        {
            error_message = "Cannot create " + temporary_path.string();
            return false;
        }

        pcd_file <<
            "# .PCD v0.7 - Point Cloud Data file format\n"
            "VERSION 0.7\n"
            "FIELDS x y z intensity time_offset_sec scan_id timestamp_sec\n"
            "SIZE 4 4 4 4 4 4 8\n"
            "TYPE F F F F F U F\n"
            "COUNT 1 1 1 1 1 1 1\n"
            "WIDTH " << chunk.points.size() << "\n"
            "HEIGHT 1\n"
            "VIEWPOINT 0 0 0 1 0 0 0\n"
            "POINTS " << chunk.points.size() << "\n"
            "DATA binary\n";

        if (!chunk.points.empty())
        {
            pcd_file.write(
                reinterpret_cast<const char *>(chunk.points.data()),
                static_cast<std::streamsize>(
                    chunk.points.size() * sizeof(TemporalObservationPoint)));
        }
        pcd_file.close();

        if (!pcd_file.good())
        {
            std::error_code cleanup_error;
            fs::remove(temporary_path, cleanup_error);
            error_message = "Failed while writing " + output_path.string();
            return false;
        }

        std::error_code rename_error;
        fs::rename(temporary_path, output_path, rename_error);
        if (rename_error)
        {
            std::error_code cleanup_error;
            fs::remove(temporary_path, cleanup_error);
            error_message =
                "Cannot move temporal chunk to final path: " +
                rename_error.message();
            return false;
        }

        {
            std::ofstream scans_file(
                fs::path(temporal_output_dir_) / "scans.csv",
                std::ios::out | std::ios::app);
            if (!scans_file.is_open())
            {
                error_message = "Cannot append temporal scans.csv";
                return false;
            }
            for (const std::string & row : chunk.scan_rows)
            {
                scans_file << row << '\n';
            }
            if (!scans_file.good())
            {
                error_message = "Failed while appending temporal scans.csv";
                return false;
            }
        }

        if (temporal_write_trajectory_knots_)
        {
            std::ofstream trajectory_file(
                fs::path(temporal_output_dir_) / "trajectory.csv",
                std::ios::out | std::ios::app);
            if (!trajectory_file.is_open())
            {
                error_message = "Cannot append temporal trajectory.csv";
                return false;
            }
            for (const std::string & row : chunk.trajectory_rows)
            {
                trajectory_file << row << '\n';
            }
            if (!trajectory_file.good())
            {
                error_message = "Failed while appending temporal trajectory.csv";
                return false;
            }
        }

        {
            std::ofstream chunks_file(
                fs::path(temporal_output_dir_) / "chunks.csv",
                std::ios::out | std::ios::app);
            if (!chunks_file.is_open())
            {
                error_message = "Cannot append temporal chunks.csv";
                return false;
            }
            chunks_file << chunk.chunk_index << ',' << file_name << ',' <<
                chunk.first_scan_id << ',' << chunk.last_scan_id << ',' <<
                chunk.scan_count << ',' << chunk.points.size() << '\n';
            if (!chunks_file.good())
            {
                error_message = "Failed while appending temporal chunks.csv";
                return false;
            }
        }

        return true;
    }

    void temporal_writer_loop()
    {
        while (true)
        {
            TemporalExportChunk chunk;
            {
                std::unique_lock<std::mutex> lock(temporal_writer_mutex_);
                temporal_writer_cv_.wait(lock, [this]() {
                    return temporal_writer_stop_ || !temporal_pending_chunks_.empty();
                });

                if (temporal_writer_stop_ && temporal_pending_chunks_.empty())
                {
                    break;
                }

                chunk = std::move(temporal_pending_chunks_.front());
                temporal_pending_chunks_.pop_front();
                temporal_writer_busy_ = true;
                temporal_writer_done_cv_.notify_all();
            }

            std::string error_message;
            const bool write_ok = write_temporal_chunk(chunk, error_message);

            {
                std::lock_guard<std::mutex> lock(temporal_writer_mutex_);
                temporal_writer_busy_ = false;
                if (write_ok)
                {
                    ++temporal_written_chunks_;
                    temporal_written_points_ += chunk.points.size();
                }
                else
                {
                    temporal_writer_failed_ = true;
                    temporal_writer_error_ = error_message;
                    temporal_pending_chunks_.clear();
                }
                temporal_writer_done_cv_.notify_all();
            }

            if (!write_ok)
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Temporal export writer failed: %s",
                    error_message.c_str());
                break;
            }

            RCLCPP_INFO(
                this->get_logger(),
                "Temporal chunk %06lu written: scans=%lu, points=%zu, ids=[%lu,%lu]",
                static_cast<unsigned long>(chunk.chunk_index),
                static_cast<unsigned long>(chunk.scan_count),
                chunk.points.size(),
                static_cast<unsigned long>(chunk.first_scan_id),
                static_cast<unsigned long>(chunk.last_scan_id));
        }
    }

    bool enqueue_active_temporal_chunk(std::string & error_message)
    {
        if (temporal_active_chunk_.scan_count == 0U)
        {
            return true;
        }

        std::unique_lock<std::mutex> lock(temporal_writer_mutex_);
        temporal_writer_done_cv_.wait(lock, [this]() {
            return temporal_writer_failed_ || temporal_writer_stop_ ||
                temporal_pending_chunks_.size() <
                    static_cast<std::size_t>(temporal_max_pending_chunks_);
        });

        if (temporal_writer_failed_)
        {
            error_message = temporal_writer_error_;
            return false;
        }
        if (temporal_writer_stop_)
        {
            error_message = "Temporal writer is already stopping.";
            return false;
        }

        temporal_active_chunk_.chunk_index = temporal_next_chunk_index_++;
        temporal_pending_chunks_.push_back(std::move(temporal_active_chunk_));
        temporal_active_chunk_ = TemporalExportChunk{};
        lock.unlock();
        temporal_writer_cv_.notify_one();
        return true;
    }

    bool flush_temporal_export(std::string & message)
    {
        if (!temporal_export_enabled_)
        {
            message = "Temporal export disabled.";
            return true;
        }

        std::string enqueue_error;
        if (!enqueue_active_temporal_chunk(enqueue_error))
        {
            message = enqueue_error;
            return false;
        }

        std::unique_lock<std::mutex> lock(temporal_writer_mutex_);
        temporal_writer_done_cv_.wait(lock, [this]() {
            return temporal_writer_failed_ ||
                (temporal_pending_chunks_.empty() && !temporal_writer_busy_);
        });

        if (temporal_writer_failed_)
        {
            message = temporal_writer_error_;
            return false;
        }

        message =
            "Temporal observations flushed to " + temporal_output_dir_ +
            " (chunks=" + std::to_string(temporal_written_chunks_) +
            ", scans=" + std::to_string(temporal_exported_scans_) +
            ", points=" + std::to_string(temporal_written_points_) + ").";
        return true;
    }

    bool shutdown_temporal_export(std::string & message)
    {
        const bool flush_ok = flush_temporal_export(message);

        {
            std::lock_guard<std::mutex> lock(temporal_writer_mutex_);
            temporal_writer_stop_ = true;
        }
        temporal_writer_cv_.notify_all();
        temporal_writer_done_cv_.notify_all();

        if (temporal_writer_thread_.joinable())
        {
            temporal_writer_thread_.join();
        }

        return flush_ok;
    }

    static void append_transform_to_csv(
        std::ostringstream & row,
        const tf2::Transform & transform)
    {
        const tf2::Vector3 & translation = transform.getOrigin();
        tf2::Quaternion quaternion = transform.getRotation();
        quaternion.normalize();

        row << ',' << translation.x() << ',' << translation.y() << ',' <<
            translation.z() << ',' << quaternion.x() << ',' << quaternion.y() <<
            ',' << quaternion.z() << ',' << quaternion.w();
    }

    void append_temporal_scan(
        const std::uint64_t scan_id,
        const tf2::Transform & T_map_lio)
    {
        {
            std::lock_guard<std::mutex> lock(temporal_writer_mutex_);
            if (temporal_writer_failed_ || temporal_writer_stop_)
            {
                return;
            }
        }

        if (scan_id > static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max()))
        {
            RCLCPP_ERROR_ONCE(
                this->get_logger(),
                "Temporal export scan_id exceeded uint32 capacity. Further scans are skipped.");
            return;
        }

        const PointCloudXYZI::Ptr source_cloud =
            temporal_use_dense_cloud_ ? feats_undistort : feats_down_body;
        if (source_cloud == nullptr || source_cloud->empty())
        {
            return;
        }

        const double scan_duration_sec =
            std::max(0.0, Measures.lidar_end_time - Measures.lidar_beg_time);
        const double minimum_range_squared =
            pcd_save_min_range_ * pcd_save_min_range_;
        const double maximum_range_squared =
            pcd_save_max_range_ * pcd_save_max_range_;

        std::size_t exported_points = 0U;
        temporal_active_chunk_.points.reserve(
            temporal_active_chunk_.points.size() +
            source_cloud->points.size() /
                static_cast<std::size_t>(temporal_point_stride_) + 1U);

        for (std::size_t index = 0U; index < source_cloud->points.size(); ++index)
        {
            if ((index % static_cast<std::size_t>(temporal_point_stride_)) != 0U)
            {
                continue;
            }

            const PointType & source_point = source_cloud->points[index];
            const double range_squared =
                static_cast<double>(source_point.x) * source_point.x +
                static_cast<double>(source_point.y) * source_point.y +
                static_cast<double>(source_point.z) * source_point.z;
            if (
                !std::isfinite(range_squared) ||
                range_squared < minimum_range_squared ||
                range_squared > maximum_range_squared)
            {
                continue;
            }

            PointType point_lio_world;
            PointType point_map;
            RGBpointBodyToWorld(&source_point, &point_lio_world);
            transform_lio_world_point_to_map(point_lio_world, &point_map, T_map_lio);

            if (
                !std::isfinite(point_map.x) ||
                !std::isfinite(point_map.y) ||
                !std::isfinite(point_map.z) ||
                !std::isfinite(point_map.intensity))
            {
                continue;
            }

            double time_offset_sec =
                static_cast<double>(source_point.curvature) / 1000.0;
            if (!std::isfinite(time_offset_sec))
            {
                continue;
            }
            time_offset_sec = std::clamp(time_offset_sec, 0.0, scan_duration_sec);

            TemporalObservationPoint output_point{};
            output_point.x = point_map.x;
            output_point.y = point_map.y;
            output_point.z = point_map.z;
            output_point.intensity = point_map.intensity;
            output_point.time_offset_sec = static_cast<float>(time_offset_sec);
            output_point.scan_id = static_cast<std::uint32_t>(scan_id);
            output_point.timestamp_sec = Measures.lidar_beg_time + time_offset_sec;
            temporal_active_chunk_.points.push_back(output_point);
            ++exported_points;
        }

        if (exported_points == 0U)
        {
            return;
        }

        ImuProcess::TimedLidarPoseVector trajectory =
            p_imu->corrected_lidar_trajectory(state_point);

        std::vector<tf2::Transform> map_trajectory;
        map_trajectory.reserve(
            std::max<std::size_t>(trajectory.size(), std::size_t{1}));
        for (const ImuProcess::TimedLidarPose & pose : trajectory)
        {
            map_trajectory.push_back(
                T_map_lio * make_transform_from_eigen(
                    pose.rotation_lio_lidar,
                    pose.position_lio_lidar));
        }

        if (map_trajectory.empty())
        {
            map_trajectory.push_back(T_map_lio * make_lio_tracking_transform());
        }

        if (temporal_write_trajectory_knots_)
        {
            for (std::size_t knot_index = 0U; knot_index < map_trajectory.size(); ++knot_index)
            {
                const double offset_sec = trajectory.empty() ? scan_duration_sec :
                    std::clamp(
                        trajectory[knot_index].offset_time_sec,
                        0.0,
                        scan_duration_sec);

                std::ostringstream trajectory_row;
                trajectory_row << std::setprecision(17) << scan_id << ',' <<
                    knot_index << ',' << offset_sec << ',' <<
                    (Measures.lidar_beg_time + offset_sec);
                append_transform_to_csv(trajectory_row, map_trajectory[knot_index]);
                temporal_active_chunk_.trajectory_rows.push_back(trajectory_row.str());
            }
        }

        std::ostringstream scan_row;
        scan_row << std::setprecision(17) << scan_id << ',' <<
            Measures.lidar_beg_time << ',' << Measures.lidar_end_time << ',' <<
            source_cloud->points.size() << ',' << exported_points << ',' <<
            map_trajectory.size();
        append_transform_to_csv(scan_row, map_trajectory.front());
        append_transform_to_csv(scan_row, map_trajectory.back());
        temporal_active_chunk_.scan_rows.push_back(scan_row.str());

        if (temporal_active_chunk_.scan_count == 0U)
        {
            temporal_active_chunk_.first_scan_id = scan_id;
        }
        temporal_active_chunk_.last_scan_id = scan_id;
        ++temporal_active_chunk_.scan_count;
        ++temporal_exported_scans_;
        temporal_exported_points_ += exported_points;

        if (
            temporal_active_chunk_.scan_count >=
            static_cast<std::uint64_t>(temporal_scans_per_chunk_))
        {
            std::string enqueue_error;
            if (!enqueue_active_temporal_chunk(enqueue_error))
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Cannot enqueue temporal export chunk: %s",
                    enqueue_error.c_str());
            }
        }
    }

    void accumulate_map_for_save(const tf2::Transform & T_map_lio)
    {
        if (!pcd_save_en && !temporal_export_enabled_)
        {
            return;
        }

        const std::uint64_t temporal_scan_id = temporal_source_scan_counter_++;
        if (
            temporal_export_enabled_ &&
            (temporal_scan_id % static_cast<std::uint64_t>(temporal_scan_stride_)) == 0U)
        {
            append_temporal_scan(temporal_scan_id, T_map_lio);
        }

        if (!pcd_save_en)
        {
            return;
        }

        ++pcd_save_seen_scans_;
        if (((pcd_save_seen_scans_ - 1U) % static_cast<std::uint64_t>(pcd_save_scan_stride_)) != 0U)
        {
            return;
        }

        const PointCloudXYZI::Ptr source_cloud =
            pcd_save_use_dense_cloud_ ? feats_undistort : feats_down_body;

        if (source_cloud == nullptr || source_cloud->empty())
        {
            return;
        }

        const double minimum_range_squared = pcd_save_min_range_ * pcd_save_min_range_;
        const double maximum_range_squared = pcd_save_max_range_ * pcd_save_max_range_;
        bool reached_capacity = false;

        for (const PointType & source_point : source_cloud->points)
        {
            const double range_squared =
                static_cast<double>(source_point.x) * source_point.x +
                static_cast<double>(source_point.y) * source_point.y +
                static_cast<double>(source_point.z) * source_point.z;

            if (
                !std::isfinite(range_squared) ||
                range_squared < minimum_range_squared ||
                range_squared > maximum_range_squared)
            {
                continue;
            }

            PointType point_lio_world;
            PointType point_map;
            RGBpointBodyToWorld(&source_point, &point_lio_world);
            transform_lio_world_point_to_map(
                point_lio_world,
                &point_map,
                T_map_lio);

            if (
                !std::isfinite(point_map.x) ||
                !std::isfinite(point_map.y) ||
                !std::isfinite(point_map.z) ||
                !std::isfinite(point_map.intensity))
            {
                continue;
            }

            const SavedMapVoxelKey key{
                static_cast<std::int64_t>(std::floor(point_map.x / pcd_save_voxel_size_)),
                static_cast<std::int64_t>(std::floor(point_map.y / pcd_save_voxel_size_)),
                static_cast<std::int64_t>(std::floor(point_map.z / pcd_save_voxel_size_))};

            auto voxel = saved_map_voxels_.find(key);
            if (voxel == saved_map_voxels_.end())
            {
                if (
                    pcd_save_max_voxels_ > 0 &&
                    saved_map_voxels_.size() >=
                        static_cast<std::size_t>(pcd_save_max_voxels_))
                {
                    reached_capacity = true;
                    continue;
                }

                voxel = saved_map_voxels_.emplace(
                    key,
                    SavedMapVoxelAccumulator{}).first;
            }

            SavedMapVoxelAccumulator & accumulator = voxel->second;
            accumulator.x_sum += point_map.x;
            accumulator.y_sum += point_map.y;
            accumulator.z_sum += point_map.z;
            accumulator.intensity_sum += point_map.intensity;
            ++accumulator.count;
            ++pcd_save_accepted_points_;
        }

        if (!saved_map_voxels_.empty())
        {
            saved_map_dirty_ = true;
        }

        if (reached_capacity)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                10000,
                "Saved-map voxel limit reached (%zu). Existing voxels continue to be "
                "updated, but new areas are no longer added. Increase "
                "pcd_save.max_voxels or pcd_save.voxel_size.",
                saved_map_voxels_.size());
        }
    }

    bool save_accumulated_map(std::string & message)
    {
        if (!pcd_save_en)
        {
            message = "Map save disabled by pcd_save.pcd_save_en.";
            return false;
        }

        if (saved_map_voxels_.empty())
        {
            message = "No accumulated map points are available yet.";
            return false;
        }

        namespace fs = std::filesystem;
        const fs::path output_path(map_file_path);
        std::string extension = output_path.extension().string();
        std::transform(
            extension.begin(),
            extension.end(),
            extension.begin(),
            [](const unsigned char value) {
                return static_cast<char>(std::tolower(value));
            });

        if (extension != ".pcd" && extension != ".ply")
        {
            message = "map_file_path must end in .pcd or .ply.";
            return false;
        }

        std::error_code directory_error;
        if (!output_path.parent_path().empty())
        {
            fs::create_directories(output_path.parent_path(), directory_error);
        }

        if (directory_error)
        {
            message =
                "Cannot create map output directory: " +
                directory_error.message();
            return false;
        }

        pcl::PointCloud<pcl::PointXYZI> output_cloud;
        output_cloud.points.reserve(saved_map_voxels_.size());

        for (const auto & entry : saved_map_voxels_)
        {
            const SavedMapVoxelAccumulator & accumulator = entry.second;
            if (accumulator.count == 0U)
            {
                continue;
            }

            const double inverse_count =
                1.0 / static_cast<double>(accumulator.count);

            pcl::PointXYZI point;
            point.x = static_cast<float>(accumulator.x_sum * inverse_count);
            point.y = static_cast<float>(accumulator.y_sum * inverse_count);
            point.z = static_cast<float>(accumulator.z_sum * inverse_count);
            point.intensity = static_cast<float>(
                accumulator.intensity_sum * inverse_count);
            output_cloud.points.push_back(point);
        }

        output_cloud.width = static_cast<std::uint32_t>(output_cloud.points.size());
        output_cloud.height = 1U;
        output_cloud.is_dense = false;

        fs::path temporary_path = output_path;
        temporary_path += ".tmp";

        RCLCPP_INFO(
            this->get_logger(),
            "Writing %zu voxelized points to %s...",
            output_cloud.points.size(),
            output_path.string().c_str());

        int result = -1;
        if (extension == ".pcd")
        {
            result = pcl::io::savePCDFileBinary(
                temporary_path.string(),
                output_cloud);
        }
        else
        {
            result = pcl::io::savePLYFileBinary(
                temporary_path.string(),
                output_cloud);
        }

        if (result != 0)
        {
            message =
                "PCL failed to write the map to " + output_path.string() + ".";
            return false;
        }

        std::error_code rename_error;
        fs::rename(temporary_path, output_path, rename_error);
        if (rename_error)
        {
            std::error_code remove_error;
            fs::remove(output_path, remove_error);
            rename_error.clear();
            fs::rename(temporary_path, output_path, rename_error);
        }

        if (rename_error)
        {
            std::error_code cleanup_error;
            fs::remove(temporary_path, cleanup_error);
            message =
                "Map data was written, but the temporary file could not be moved "
                "to the final path: " + rename_error.message();
            return false;
        }

        saved_map_dirty_ = false;
        message =
            "Map saved to " + output_path.string() + " with " +
            std::to_string(output_cloud.points.size()) +
            " voxelized points from " +
            std::to_string(pcd_save_accepted_points_) +
            " accepted observations.";
        return true;
    }

    bool save_all_exports(std::string & message)
    {
        bool any_export_enabled = false;
        bool success = true;
        bool has_message = false;
        std::ostringstream combined_message;

        if (pcd_save_en)
        {
            any_export_enabled = true;
            std::string map_message;
            const bool map_success = save_accumulated_map(map_message);
            success = success && map_success;
            combined_message << map_message;
            has_message = true;
        }

        if (temporal_export_enabled_)
        {
            any_export_enabled = true;
            std::string temporal_message;
            const bool temporal_success = flush_temporal_export(temporal_message);
            success = success && temporal_success;
            if (has_message)
            {
                combined_message << ' ';
            }
            combined_message << temporal_message;
            has_message = true;
        }

        if (!any_export_enabled)
        {
            message = "Both compact map saving and temporal export are disabled.";
            return false;
        }

        message = combined_message.str();
        return success;
    }

    void map_save_callback(
        std_srvs::srv::Trigger::Request::ConstSharedPtr request,
        std_srvs::srv::Trigger::Response::SharedPtr response)
    {
        static_cast<void>(request);
        response->success = save_all_exports(response->message);

        if (response->success)
        {
            RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        }
    }

    void publish_map_to_odom_tf(const tf2::Transform & T_map_tracking, const tf2::Transform & T_odom_tracking) {
        if (!rep105_enable_ || !rep105_publish_map_to_odom_tf_) {
            return;
        }

        const tf2::Transform T_map_odom =
            T_map_tracking * T_odom_tracking.inverse();

        geometry_msgs::msg::TransformStamped map_to_odom_msg;
        map_to_odom_msg.header.stamp = get_ros_time(lidar_end_time);
        map_to_odom_msg.header.frame_id = rep105_map_frame_;
        map_to_odom_msg.child_frame_id = rep105_odom_frame_;
        map_to_odom_msg.transform = tf2::toMsg(T_map_odom);

        tf_broadcaster_->sendTransform(map_to_odom_msg);
    }

    void queue_stats_callback(){
        std::size_t lidar_queue_size = 0;
        std::size_t imu_queue_size = 0;

        std::size_t lidar_high_watermark_interval = 0;
        std::size_t imu_high_watermark_interval = 0;

        std::size_t lidar_high_watermark_total = 0;
        std::size_t imu_high_watermark_total = 0;

        std::uint64_t lidar_received = 0;
        std::uint64_t imu_received = 0;
        std::uint64_t synced_scans = 0;

        std::uint64_t wait_for_imu_polls_interval = 0;
        std::uint64_t wait_for_imu_polls_total = 0;

        std::uint64_t lidar_timestamp_resets = 0;
        std::uint64_t imu_timestamp_resets = 0;

        double lidar_queue_span_sec = 0.0;
        double imu_queue_span_sec = 0.0;

        bool waiting_for_imu = false;

        {
            std::lock_guard<std::mutex> lock(mtx_buffer);
            lidar_queue_size = lidar_buffer.size();
            imu_queue_size = imu_buffer.size();
            lidar_high_watermark_interval = queue_diagnostics.lidar_high_watermark_interval;
            imu_high_watermark_interval = queue_diagnostics.imu_high_watermark_interval;
            lidar_high_watermark_total = queue_diagnostics.lidar_high_watermark_total;
            imu_high_watermark_total = queue_diagnostics.imu_high_watermark_total;
            lidar_received = queue_diagnostics.lidar_received;
            imu_received = queue_diagnostics.imu_received;
            synced_scans = queue_diagnostics.synced_scans;
            wait_for_imu_polls_interval = queue_diagnostics.wait_for_imu_polls_interval;
            wait_for_imu_polls_total = queue_diagnostics.wait_for_imu_polls_total;
            lidar_timestamp_resets = queue_diagnostics.lidar_timestamp_resets;
            imu_timestamp_resets = queue_diagnostics.imu_timestamp_resets;

            if (!time_buffer.empty())
            {
                lidar_queue_span_sec = std::max(0.0, last_timestamp_lidar - time_buffer.front());
            }

            if (!imu_buffer.empty())
            {
                const double oldest_imu_timestamp = get_time_sec(imu_buffer.front()->header.stamp);
                imu_queue_span_sec = std::max(0.0, last_timestamp_imu - oldest_imu_timestamp);
            }

            waiting_for_imu = lidar_pushed && last_timestamp_imu < lidar_end_time;

            // Reiniciar únicamente las estadísticas del periodo.
            // Los máximos históricos se conservan.
            queue_diagnostics.lidar_high_watermark_interval = lidar_queue_size;

            queue_diagnostics.imu_high_watermark_interval = imu_queue_size;

            queue_diagnostics.wait_for_imu_polls_interval = 0;
        }

        const std::uint64_t mapping_completed = mapping_completed_count.load(std::memory_order_relaxed);
        const auto current_time = std::chrono::steady_clock::now();
        const double elapsed_sec = std::chrono::duration<double>(current_time - previous_queue_stats_time_).count();

        if (elapsed_sec <= 0.0) return;

        const double lidar_receive_rate = static_cast<double>(lidar_received - previous_lidar_received_) / elapsed_sec;
        const double imu_receive_rate =static_cast<double>(imu_received - previous_imu_received_) / elapsed_sec;
        const double sync_rate = static_cast<double>(synced_scans - previous_synced_scans_) / elapsed_sec;
        const double mapping_rate = static_cast<double>(mapping_completed - previous_mapping_completed_) /elapsed_sec;

        previous_lidar_received_ = lidar_received;
        previous_imu_received_ = imu_received;
        previous_synced_scans_ = synced_scans;
        previous_mapping_completed_ = mapping_completed;
        previous_queue_stats_time_ = current_time;

        const bool queue_warning =
            lidar_queue_size >= static_cast<std::size_t>(lidar_queue_warn_size_) ||
            lidar_high_watermark_interval >= static_cast<std::size_t>(lidar_queue_warn_size_) ||
            imu_queue_size >= static_cast<std::size_t>(imu_queue_warn_size_) ||
            imu_high_watermark_interval >= static_cast<std::size_t>(imu_queue_warn_size_);

        const char * log_format =
            "Queue stats | "
            "LiDAR: current=%zu interval_max=%zu total_max=%zu "
            "span=%.3f s rx=%.1f Hz | "
            "IMU: current=%zu interval_max=%zu total_max=%zu "
            "span=%.3f s rx=%.1f Hz | "
            "sync=%.1f Hz mapping=%.1f Hz "
            "waiting_imu=%s wait_polls=%llu "
            "wait_polls_total=%llu resets=%llu/%llu";

        if (queue_warning)
        {
            RCLCPP_WARN(
                this->get_logger(),
                log_format,
                lidar_queue_size,
                lidar_high_watermark_interval,
                lidar_high_watermark_total,
                lidar_queue_span_sec,
                lidar_receive_rate,
                imu_queue_size,
                imu_high_watermark_interval,
                imu_high_watermark_total,
                imu_queue_span_sec,
                imu_receive_rate,
                sync_rate,
                mapping_rate,
                waiting_for_imu ? "true" : "false",
                static_cast<unsigned long long>(wait_for_imu_polls_interval),
                static_cast<unsigned long long>(wait_for_imu_polls_total),
                static_cast<unsigned long long>(lidar_timestamp_resets),
                static_cast<unsigned long long>(imu_timestamp_resets));
        }
        else
        {
            RCLCPP_INFO(
                this->get_logger(),
                log_format,
                lidar_queue_size,
                lidar_high_watermark_interval,
                lidar_high_watermark_total,
                lidar_queue_span_sec,
                lidar_receive_rate,
                imu_queue_size,
                imu_high_watermark_interval,
                imu_high_watermark_total,
                imu_queue_span_sec,
                imu_receive_rate,
                sync_rate,
                mapping_rate,
                waiting_for_imu ? "true" : "false",
                static_cast<unsigned long long>(wait_for_imu_polls_interval),
                static_cast<unsigned long long>(wait_for_imu_polls_total),
                static_cast<unsigned long long>(lidar_timestamp_resets),
                static_cast<unsigned long long>(imu_timestamp_resets));
        }
    }

private:
    rclcpp::CallbackGroup::SharedPtr lidar_callback_group_;
    rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
    rclcpp::CallbackGroup::SharedPtr mapping_callback_group_;

    int executor_threads_ = 3;
    int lidar_qos_depth_ = 50;
    int imu_qos_depth_ = 400;

    bool deskew_only_ = false;
    bool queue_stats_enabled_ = true;
    double queue_stats_period_sec_ = 1.0;

    int lidar_queue_warn_size_ = 3;
    int imu_queue_warn_size_ = 100;

    std::uint64_t previous_lidar_received_ = 0;
    std::uint64_t previous_imu_received_ = 0;
    std::uint64_t previous_synced_scans_ = 0;
    std::uint64_t previous_mapping_completed_ = 0;

    std::chrono::steady_clock::time_point previous_queue_stats_time_ = std::chrono::steady_clock::now();

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_lidar_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr map_pub_timer_;
    rclcpp::TimerBase::SharedPtr queue_stats_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_srv_;

    std::unordered_map<
        SavedMapVoxelKey,
        SavedMapVoxelAccumulator,
        SavedMapVoxelKeyHash> saved_map_voxels_;
    bool pcd_save_on_shutdown_ = true;
    bool pcd_save_use_dense_cloud_ = true;
    bool saved_map_dirty_ = false;
    double pcd_save_voxel_size_ = 0.15;
    double pcd_save_min_range_ = 0.5;
    double pcd_save_max_range_ = 80.0;
    int pcd_save_scan_stride_ = 1;
    std::int64_t pcd_save_max_voxels_ = 20000000;
    std::int64_t pcd_save_reserve_voxels_ = 1000000;
    std::uint64_t pcd_save_seen_scans_ = 0U;
    std::uint64_t pcd_save_accepted_points_ = 0U;

    bool temporal_export_enabled_ = false;
    bool temporal_use_dense_cloud_ = true;
    bool temporal_write_trajectory_knots_ = true;
    std::string temporal_output_dir_;
    int temporal_scans_per_chunk_ = 200;
    int temporal_max_pending_chunks_ = 2;
    int temporal_scan_stride_ = 1;
    int temporal_point_stride_ = 1;

    TemporalExportChunk temporal_active_chunk_;
    std::deque<TemporalExportChunk> temporal_pending_chunks_;
    std::thread temporal_writer_thread_;
    std::mutex temporal_writer_mutex_;
    std::condition_variable temporal_writer_cv_;
    std::condition_variable temporal_writer_done_cv_;
    bool temporal_writer_stop_ = false;
    bool temporal_writer_busy_ = false;
    bool temporal_writer_failed_ = false;
    std::string temporal_writer_error_;

    std::uint64_t temporal_source_scan_counter_ = 0U;
    std::uint64_t temporal_next_chunk_index_ = 1U;
    std::uint64_t temporal_exported_scans_ = 0U;
    std::uint64_t temporal_exported_points_ = 0U;
    std::uint64_t temporal_written_chunks_ = 0U;
    std::uint64_t temporal_written_points_ = 0U;

    bool effect_pub_en = false, map_pub_en = false;
    bool rep105_enable_ = false;
    bool rep105_publish_map_to_odom_tf_ = false;
    bool rep105_publish_lio_tf_ = true;
    bool rep105_use_latest_robot_tf_ = true;
    bool rep105_align_map_to_odom_on_start_ = true;
    bool rep105_project_map_to_2d_ = false;
    bool rep105_initial_alignment_ready_ = false;
    bool latest_map_alignment_ready_ = false;
    double rep105_tf_timeout_sec_ = 0.05;
    tf2::Transform T_map_lio_initial_;
    tf2::Transform latest_T_map_lio_;

    std::string lio_world_frame_ = "camera_init";
    std::string lio_body_frame_ = "body";
    std::string deskewed_topic_ = "/cloud_registered_lidar";

    std::string rep105_map_frame_ = "map";
    std::string rep105_odom_frame_ = "odom";
    std::string rep105_robot_tracking_frame_ = "mid360";
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
    double epsi[23] = {0.001};

    FILE * fp = nullptr;
    ofstream fout_pre, fout_out, fout_dbg;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LaserMappingNode>();

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), node->executor_thread_count());

    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);

    // Cerrar ficheros y destruir recursos del nodo antes
    // del guardado final realizado con variables globales.
    node.reset();

    if (rclcpp::ok())
        rclcpp::shutdown();
    if (runtime_pos_log)
    {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;    
        FILE *fp2;
        string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(),"w");
        fprintf(fp2,"time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
        for (int i = 0;i<time_log_counter; i++){
            fprintf(fp2,"%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",T1[i],s_plot[i],int(s_plot2[i]),s_plot3[i],s_plot4[i],int(s_plot5[i]),s_plot6[i],int(s_plot7[i]),int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }

    return 0;
}
