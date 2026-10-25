#include "ieskf.h"
#include <iostream>

double State::gravity = 9.81;
//新增约束参数
double State::max_bias_gyro = 0.1;    
double State::max_bias_accel = 0.5;
double State::max_velocity = 10.0;

M3D Jr(const V3D &inp)
{
    return Sophus::SO3d::leftJacobian(inp).transpose();
}
M3D JrInv(const V3D &inp)
{
    return Sophus::SO3d::leftJacobianInverse(inp).transpose();
}

void State::operator+=(const V21D &delta)
{
    r_wi *= Sophus::SO3d::exp(delta.segment<3>(0)).matrix();
    t_wi += delta.segment<3>(3);
    r_il *= Sophus::SO3d::exp(delta.segment<3>(6)).matrix();
    t_il += delta.segment<3>(9);
    v += delta.segment<3>(12);
    bg += delta.segment<3>(15);
    ba += delta.segment<3>(18);
}

//新增约束函数
void State::applyConstraints() {
    
    // 约束陀螺仪偏置
    bg = bg.cwiseMax(-V3D::Ones() * max_bias_gyro)
            .cwiseMin(V3D::Ones() * max_bias_gyro);
    
    // 约束加速度计偏置  
    ba = ba.cwiseMax(-V3D::Ones() * max_bias_accel)
            .cwiseMin(V3D::Ones() * max_bias_accel);
    
    // 约束速度
    v = v.cwiseMax(-V3D::Ones() * max_velocity)
          .cwiseMin(V3D::Ones() * max_velocity);

}


V21D State::operator-(const State &other) const
{
    V21D delta = V21D::Zero();
    delta.segment<3>(0) = Sophus::SO3d(other.r_wi.transpose() * r_wi).log();
    delta.segment<3>(3) = t_wi - other.t_wi;
    delta.segment<3>(6) = Sophus::SO3d(other.r_il.transpose() * r_il).log();
    delta.segment<3>(9) = t_il - other.t_il;
    delta.segment<3>(12) = v - other.v;
    delta.segment<3>(15) = bg - other.bg;
    delta.segment<3>(18) = ba - other.ba;
    return delta;
}

std::ostream &operator<<(std::ostream &os, const State &state)
{
    os << "==============START===============" << std::endl;
    os << "r_wi: " << state.r_wi.eulerAngles(2, 1, 0).transpose() << std::endl;
    os << "t_il: " << state.t_il.transpose() << std::endl;
    os << "r_il: " << state.r_il.eulerAngles(2, 1, 0).transpose() << std::endl;
    os << "t_wi: " << state.t_wi.transpose() << std::endl;
    os << "v: " << state.v.transpose() << std::endl;
    os << "bg: " << state.bg.transpose() << std::endl;
    os << "ba: " << state.ba.transpose() << std::endl;
    os << "g: " << state.g.transpose() << std::endl;
    os << "===============END================" << std::endl;

    return os;
}

void IESKF::predict(const Input &inp, double dt, const M12D &Q)
{
    V21D delta = V21D::Zero();
    delta.segment<3>(0) = (inp.gyro - m_x.bg) * dt;
    delta.segment<3>(3) = m_x.v * dt;
    delta.segment<3>(12) = (m_x.r_wi * (inp.acc - m_x.ba) + m_x.g) * dt;

    m_F.setIdentity();
    m_F.block<3, 3>(0, 0) = Sophus::SO3d::exp(-(inp.gyro - m_x.bg) * dt).matrix();
    m_F.block<3, 3>(0, 15) = -Jr((inp.gyro - m_x.bg) * dt) * dt;
    m_F.block<3, 3>(3, 12) = Eigen::Matrix3d::Identity() * dt;
    m_F.block<3, 3>(12, 0) = -m_x.r_wi * Sophus::SO3d::hat(inp.acc - m_x.ba) * dt;
    m_F.block<3, 3>(12, 18) = -m_x.r_wi * dt;

    m_G.setZero();
    m_G.block<3, 3>(0, 0) = -Jr((inp.gyro - m_x.bg) * dt) * dt;
    m_G.block<3, 3>(12, 3) = -m_x.r_wi * dt;
    m_G.block<3, 3>(15, 6) = Eigen::Matrix3d::Identity() * dt;
    m_G.block<3, 3>(18, 9) = Eigen::Matrix3d::Identity() * dt;

    m_x += delta;
    m_P = m_F * m_P * m_F.transpose() + m_G * Q * m_G.transpose();
}

void IESKF::update()
{
    State predict_x = m_x;
    SharedState shared_data;
    shared_data.iter_num = 0;
    shared_data.res = 1e10;
    V21D delta = V21D::Zero();
    M21D H = M21D::Zero();
    V21D b;
    Eigen::LDLT<M21D> H_ldlt;
    // A scan is only allowed to shape the state and the covariance if at least one
    // iteration ended on a usable linear solve; otherwise the propagated state and
    // covariance are kept (a scan with no usable observation is not a measurement).
    bool have_update = false;
    bool last_usable = false;
    ++m_update_calls;

    for (size_t i = 0; i < m_max_iter; i++)
    {
        m_loss_func(m_x, shared_data);
        if (!shared_data.valid)
            break;
        H.setZero();
        b.setZero();
        delta = m_x - predict_x;
        M21D J = M21D::Identity();
        J.block<3, 3>(0, 0) = JrInv(delta.segment<3>(0));
        J.block<3, 3>(6, 6) = JrInv(delta.segment<3>(6));
        // m_P is symmetric: factorise it once and solve for both right-hand sides
        // instead of forming the explicit 21x21 inverse twice per iteration.
        Eigen::LDLT<M21D> P_ldlt(m_P);
        H.noalias() = J.transpose() * P_ldlt.solve(J);
        b.noalias() = J.transpose() * P_ldlt.solve(delta);

        H.block<12, 12>(0, 0) += shared_data.H;
        b.block<12, 1>(0, 0) += shared_data.b;

        H_ldlt.compute(H);
        const V21D delta_step = H_ldlt.solve(-b);
        if (P_ldlt.info() != Eigen::Success || H_ldlt.info() != Eigen::Success ||
            !H.allFinite() || !delta_step.allFinite())
        {
            // Degrade, do not crash: a non-finite/inf step would be composed into
            // r_wi/r_il and the next Sophus::SO3d construction would abort the node.
            ++m_reject_total;
            last_usable = false;
            break;
        }
        delta = delta_step;

        m_x += delta;
        shared_data.iter_num += 1;
        have_update = true;
        last_usable = true;

        if (m_stop_func(delta))
            break;
    }

    //约束应用
    m_x.applyConstraints();
    // RCLCPP_DEBUG(rclcpp::get_logger("kf_debug"), "IESKF更新完成，约束已应用");

    if (have_update && last_usable)
    {
        M21D L = M21D::Identity();
        // L.block<3, 3>(0, 0) = JrInv(delta.segment<3>(0));
        // L.block<3, 3>(6, 6) = JrInv(delta.segment<3>(6));
        L.block<3, 3>(0, 0) = Jr(delta.segment<3>(0));
        L.block<3, 3>(6, 6) = Jr(delta.segment<3>(6));
        // H_ldlt still holds the last accepted iteration's H; solve for the identity
        // rather than forming an explicit inverse. P_new = L * H^-1 * L^T is the
        // standard iterated-EKF result for this information-form update.
        m_P = L * H_ldlt.solve(M21D::Identity()) * L.transpose();
    }
    // else: no usable observation this scan -> keep the propagated covariance
    // (the previous code built m_P from H's initialiser in that case, resetting P to
    //  L*I*L^T, and would now build it from a zero matrix).

    if (m_reject_total != m_reject_reported || (m_update_calls % 100) == 0)
    {
        m_reject_reported = m_reject_total;
        std::cerr << "[ieskf] updates=" << m_update_calls << " rejected=" << m_reject_total
                  << " (total over this process)" << std::endl;
    }
}

void IESKF::updateLegVelocity(const V3D &v_body_meas, const M3D &R_cov)
{
    // Measurement model: z = v_b_meas, h(x) = R_wi^T * v_w
    V3D z_hat = m_x.r_wi.transpose() * m_x.v;
    V3D res = v_body_meas - z_hat;

    // Measurement Jacobian H: 3 x 21
    Eigen::Matrix<double, 3, 21> H = Eigen::Matrix<double, 3, 21>::Zero();
    M3D skew_v = Sophus::SO3d::hat(z_hat);
    H.block<3, 3>(0, 0) = skew_v;
    H.block<3, 3>(0, 12) = m_x.r_wi.transpose();

    // Innovation covariance: S = H * P * H^T + R_cov
    M3D S = H * m_P * H.transpose() + R_cov;
    Eigen::LDLT<M3D> S_ldlt(S);
    if (S_ldlt.info() != Eigen::Success)
        return;

    // Kalman gain: K = P * H^T * S^-1 (21 x 3)
    Eigen::Matrix<double, 21, 3> K = m_P * H.transpose() * S_ldlt.solve(M3D::Identity());

    // State error correction
    V21D delta = K * res;
    m_x += delta;
    m_x.applyConstraints();

    // Joseph-form covariance update: P = (I - K*H)*P*(I - K*H)^T + K*R*K^T
    M21D I_KH = M21D::Identity() - K * H;
    m_P = I_KH * m_P * I_KH.transpose() + K * R_cov * K.transpose();
}

void IESKF::updateZUPT(double cov)
{
    updateLegVelocity(V3D::Zero(), M3D::Identity() * cov);
}