#pragma once
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>

using PointType = pcl::PointXYZINormal;
using CloudType = pcl::PointCloud<PointType>;
using PointVec = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

using M3D = Eigen::Matrix3d;
using V3D = Eigen::Vector3d;
using M3F = Eigen::Matrix3f;
using V3F = Eigen::Vector3f;
using M2D = Eigen::Matrix2d;
using V2D = Eigen::Vector2d;
using M2F = Eigen::Matrix2f;
using V2F = Eigen::Vector2f;
using M4D = Eigen::Matrix4d;
using V4D = Eigen::Vector4d;

template <typename T>
using Vec = std::vector<T>;

bool esti_plane(PointVec &points, const double &thresh, V4D &out);

float sq_dist(const PointType &p1, const PointType &p2);

struct Config
{
    int lidar_filter_num = 3;
    double lidar_min_range = 0.5;
    double lidar_max_range = 20.0;
    double scan_resolution = 0.15;
    double map_resolution = 0.3;

    std::string lidar_type = "livox"; // "livox"(CustomMsg) 或 "pointcloud2"(通用 PointCloud2)
    int lidar_max_line = 4;           // Livox 扫描线数(MID360=4)；pointcloud2 模式下忽略
    double imu_acc_scale = 10.0;      // IMU 加速度缩放(MID360 内置 IMU 单位约为 g，需 ×10 近 m/s²；标准 IMU 设 1.0)

    double cube_len = 300;
    double det_range = 60;
    double move_thresh = 1.5;

    double na = 0.01;
    double ng = 0.01;
    double nba = 0.0001;
    double nbg = 0.0001;
    int imu_init_num = 20;
    // Static-window IMU init (round 2): when imu_init_window_s > 0, initialization waits for a
    // stationary window of that length (gyro/accel variance thresholds) instead of a fixed sample
    // count, so bg/gravity come from the full quiet period rather than its first 0.25 s.
    double imu_init_window_s = 0.0;
    double imu_init_static_gyro_std = 0.005; // rad/s per axis
    double imu_init_static_acc_dev = 0.3;    // m/s^2 per axis deviation from window mean
    double imu_init_max_wait_s = 15.0;       // fall back to the quietest window after this
    int near_search_num = 5;
    int ieskf_max_iter = 5;
    bool gravity_align = true;
    bool esti_il = false;
    double point_quality_thresh = 0.1; // 点平面残差质量阈值，与上游 FAST-LIO2 一致
    M3D r_il = M3D::Identity();
    V3D t_il = V3D::Zero();
    M3D r_cl = M3D::Identity();
    V3D t_cl = V3D::Zero();

    double cam_width = 1280;
    double cam_height = 720;
    double cam_fx = 641.976318359375;
    double cam_fy = 641.0658569335938;
    double cam_cx = 641.5721435546875;
    double cam_cy = 368.21832275390625;
    Vec<double> cam_d{-0.05588280409574509, 0.06619580090045929, 0.0002959131670650095, 0.0008284428040497005, -0.021595485508441925};

    double lidar_cov_inv = 1000.0;

    // 添加状态约束参数
    double max_bias_gyro = 0.1;
    double max_bias_accel = 0.2; 
    double max_velocity = 10.0;
};

struct IMUData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    V3D acc;
    V3D gyro;
    double time;
    IMUData() = default;
    IMUData(const V3D &a, const V3D &g, double &t) : acc(a), gyro(g), time(t) {}
};

struct Pose
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double offset;
    V3D acc;
    V3D gyro;
    V3D vel;
    V3D trans;
    M3D rot;
    Pose() = default;
    Pose(double t, const V3D &a, const V3D &g, const V3D &v, const V3D &p, const M3D &r) : offset(t), acc(a), gyro(g), vel(v), trans(p), rot(r) {}
};

struct SyncPackage
{
    Vec<IMUData> imus;
    CloudType::Ptr cloud;
    double cloud_start_time = 0.0;
    double cloud_end_time = 0.0;
    double image_time = 0.0;
    bool lidar_end;
    cv::Mat image;
};