#pragma once
#include <Eigen/Eigen>
#include <sophus/so3.hpp>
#include "commons.h"
#include <chrono>
#include <rclcpp/logging.hpp>

using M12D = Eigen::Matrix<double, 12, 12>;
using M21D = Eigen::Matrix<double, 21, 21>;

using V12D = Eigen::Matrix<double, 12, 1>;
using V21D = Eigen::Matrix<double, 21, 1>;
using M21X12D = Eigen::Matrix<double, 21, 12>;

M3D Jr(const V3D &inp);
M3D JrInv(const V3D &inp);

struct SharedState
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    M12D H;
    V12D b;
    double res = 1e10;
    bool valid = false;
    size_t iter_num = 0;
};
struct Input
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    V3D acc;
    V3D gyro;
    Input() = default;
    Input(V3D &a, V3D &g) : acc(a), gyro(g) {}
    Input(double a1, double a2, double a3, double g1, double g2, double g3) : acc(a1, a2, a3), gyro(g1, g2, g3) {}
};
struct State
{
    static double gravity;
    M3D r_wi = M3D::Identity();
    V3D t_wi = V3D::Zero();
    M3D r_il = M3D::Identity();
    V3D t_il = V3D::Zero();
    V3D v = V3D::Zero();
    V3D bg = V3D::Zero();
    V3D ba = V3D::Zero();
    V3D g = V3D(0.0, 0.0, -9.81);

    // 添加偏置约束参数
    static double max_bias_gyro;    // 陀螺仪偏置最大值
    static double max_bias_accel;   // 加速度计偏置最大值
    static double max_velocity;     // 速度最大值

    void initGravityDir(const V3D &gravity_dir) { g = gravity_dir.normalized() * State::gravity; }

    void operator+=(const V21D &delta);

    V21D operator-(const State &other) const;

    friend std::ostream &operator<<(std::ostream &os, const State &state);

    //新增约束函数
    void applyConstraints();
};

using loss_func = std::function<void(State &, SharedState &)>;
using stop_func = std::function<bool(const V21D &)>;

class IESKF
{
public:
    // ==================== 诊断代码开始 ====================
    // 删除默认构造函数声明，添加带logger的构造函数
    IESKF(rclcpp::Logger logger = rclcpp::get_logger("ieskf")) : m_logger(logger) {}
    // ==================== 诊断代码结束 ====================

    
    // ==================== 诊断代码开始 ====================
    void setLogger(rclcpp::Logger logger) { m_logger = logger; }
    // ==================== 诊断代码结束 ====================
    void setMaxIter(size_t iter) { m_max_iter = iter; }
    void setLossFunction(loss_func func) { m_loss_func = func; }
    void setStopFunction(stop_func func) { m_stop_func = func; }

    void predict(const Input &inp, double dt, const M12D &Q);

    void update();
    void updateLegVelocity(const V3D &v_body_meas, const M3D &R_cov);
    void updateZUPT(double cov = 1e-4);

    State &x() { return m_x; }

    M21D &P() { return m_P; }

private:
    size_t m_max_iter = 10;
    // audit counters for the graceful-degradation path in update()
    size_t m_update_calls = 0;
    size_t m_reject_total = 0;
    size_t m_reject_reported = 0;
    State m_x;
    M21D m_P;
    loss_func m_loss_func;
    stop_func m_stop_func;
    M21D m_F;
    M21X12D m_G;
    // ==================== 诊断代码开始 ====================
    rclcpp::Logger m_logger;
    // ==================== 诊断代码结束 ====================s
};
