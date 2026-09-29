#include "imu_processor.h"
#include "imu_init.h"

IMUProcessor::IMUProcessor(Config &config, std::shared_ptr<IESKF> kf) : m_config(config), m_kf(kf), m_logger(rclcpp::get_logger("imu_processor"))
{
    m_Q.setIdentity();
    m_Q.block<3, 3>(0, 0) = M3D::Identity() * m_config.ng;
    m_Q.block<3, 3>(3, 3) = M3D::Identity() * m_config.na;
    m_Q.block<3, 3>(6, 6) = M3D::Identity() * m_config.nbg;
    m_Q.block<3, 3>(9, 9) = M3D::Identity() * m_config.nba;
    m_last_acc.setZero();
    m_last_gyro.setZero();
    m_imu_cache.clear();
    m_poses_cache.clear();
    m_pushed = true;
}

bool IMUProcessor::initialize(SyncPackage &package)
{
    m_imu_cache.insert(m_imu_cache.end(), package.imus.begin(), package.imus.end());
    // Window selection + readiness verdict.  Legacy = whole cache once imu_init_num samples
    // are buffered; static-window mode = the quietest contiguous imu_init_window_s inside the
    // cache, with a TIME-based fallback after imu_init_max_wait_s (see imu_init.h; the fallback
    // must not additionally require a quiet accel or init deadlocks, review item B3).
    const ImuInitPlan plan = planImuInit(m_imu_cache, m_config.imu_init_window_s,
                                         m_config.imu_init_max_wait_s,
                                         m_config.imu_init_static_gyro_std,
                                         m_config.imu_init_static_acc_dev,
                                         m_config.imu_init_num);
    if (!plan.ready)
        return false;
    const size_t i0 = plan.i0, i1 = plan.i1;
    const size_t n = m_imu_cache.size();
    if (plan.use_static_window)
        RCLCPP_WARN(m_logger, "IMU init window [%zu,%zu)/%zu gyro_std=%.5f acc_dev=%.4f static=%d waited_out=%d",
                    i0, i1, n, plan.gyro_std, plan.acc_dev, (int)plan.is_static, (int)plan.waited_out);
    if (plan.waited_out)
        RCLCPP_WARN(m_logger, "IMU init: no static window within %.1fs; falling back to the quietest window (ends %.3fs before the newest IMU sample) with v=0 at its end",
                    m_config.imu_init_max_wait_s, m_imu_cache.back().time - m_imu_cache[i1 - 1].time);

    V3D acc_mean = V3D::Zero();
    V3D gyro_mean = V3D::Zero();
    for (size_t k = i0; k < i1; ++k)
    {
        acc_mean += m_imu_cache[k].acc;
        gyro_mean += m_imu_cache[k].gyro;
    }
    acc_mean /= static_cast<double>(i1 - i0);
    gyro_mean /= static_cast<double>(i1 - i0);
    m_kf->x().r_il = m_config.r_il;
    m_kf->x().t_il = m_config.t_il;
    m_kf->x().bg = gyro_mean;
    if (m_config.gravity_align)
    {
        // The propagation model r_wi*(acc - ba) + g must null exactly at rest: the
        // gravity magnitude has to match the specific force the IMU actually reports,
        // not a textbook 9.81. Datasets differ by up to ~3% (BMI085/VN200 scale error
        // or local g), which otherwise becomes a constant accel leak the accel-bias
        // state must absorb (clamped) every scan, corrupting fast-motion prediction.
        // Only adopt the measured magnitude when the init window is genuinely static - and
        // measure the deviation over THAT window (not the whole cache, whose tail may be
        // seconds of motion in waited-out mode) with the configured threshold (review N1).
        const double g_meas = acc_mean.norm();
        const bool window_static = plan.is_static || !plan.use_static_window;
        if (g_meas > 8.5 && g_meas < 10.5 && window_static &&
            plan.acc_dev < m_config.imu_init_static_acc_dev)
            State::gravity = g_meas;
        m_kf->x().r_wi = (Eigen::Quaterniond::FromTwoVectors((-acc_mean).normalized(), V3D(0.0, 0.0, -1.0)).matrix());
        m_kf->x().initGravityDir(V3D(0, 0, -1.0));
        RCLCPP_WARN(m_logger, "IMU init: g=%.4f (acc_dev_max=%.4f) bg=[%.5f %.5f %.5f] acc_mean=[%.4f %.4f %.4f]",
                    State::gravity, plan.acc_dev,
                    gyro_mean.x(), gyro_mean.y(), gyro_mean.z(),
                    acc_mean.x(), acc_mean.y(), acc_mean.z());
    }
    else
        m_kf->x().initGravityDir(-acc_mean);
    m_kf->P().setIdentity();
    m_kf->P().block<3, 3>(6, 6) = M3D::Identity() * 0.00001;
    m_kf->P().block<3, 3>(9, 9) = M3D::Identity() * 0.00001;
    m_kf->P().block<3, 3>(15, 15) = M3D::Identity() * 0.0001;
    m_kf->P().block<3, 3>(18, 18) = M3D::Identity() * 0.0001;

    // The state above (r_wi, bg, gravity) belongs to the END of the chosen analysis window,
    // but m_last_imu is the NEWEST buffered sample.  Jumping to it used to drop the rotation
    // and velocity of [i1, n) - seconds of it in the waited-out fallback - and to invent a
    // stationary platform at the current time (review N2).  Propagate the remainder so the
    // state really is at m_imu_cache.back().  v = 0 at the window end remains the assumption
    // of a static window; its covariance (P block 12, left at the identity) stays large
    // enough for the filter to correct it as soon as a scan observes the motion.
    for (size_t k = i1; k > 0 && k < m_imu_cache.size(); ++k)
    {
        const IMUData &head = m_imu_cache[k - 1];
        const IMUData &tail = m_imu_cache[k];
        Input inp;
        inp.gyro = 0.5 * (head.gyro + tail.gyro);
        inp.acc = 0.5 * (head.acc + tail.acc);
        m_kf->predict(inp, tail.time - head.time, m_Q);
    }

    m_last_imu = m_imu_cache.back();
    // The state now lives at the last buffered sample, not at the end of the lidar frame.
    // undistort() propagates from here, so the last interval is not counted twice: the pair
    // whose tail is exactly this timestamp gets dt = 0 (tail - end).
    m_last_propagate_end_time = m_last_imu.time;
    return true;
}

void IMUProcessor::undistort(SyncPackage &package)
{

    // ==================== 诊断代码开始 ====================
    // 监控IMU数据范围
    if (!m_imu_cache.empty()) {
        V3D acc_mean = V3D::Zero();
        V3D gyro_mean = V3D::Zero();
        V3D acc_max = V3D::Zero();
        V3D gyro_max = V3D::Zero();
        
        for (const auto& imu : m_imu_cache) {
            acc_mean += imu.acc;
            gyro_mean += imu.gyro;
            acc_max = acc_max.cwiseMax(imu.acc.cwiseAbs());
            gyro_max = gyro_max.cwiseMax(imu.gyro.cwiseAbs());
        }
        acc_mean /= m_imu_cache.size();
        gyro_mean /= m_imu_cache.size();
        
        // RCLCPP_INFO(m_logger, 
        //     "IMU_STATS: Count=%zu, AccMean=[%.3f,%.3f,%.3f], GyroMean=[%.3f,%.3f,%.3f], "
        //     "AccMax=[%.3f,%.3f,%.3f], GyroMax=[%.3f,%.3f,%.3f]",
        //     m_imu_cache.size(), 
        //     acc_mean.x(), acc_mean.y(), acc_mean.z(),
        //     gyro_mean.x(), gyro_mean.y(), gyro_mean.z(),
        //     acc_max.x(), acc_max.y(), acc_max.z(),
        //     gyro_max.x(), gyro_max.y(), gyro_max.z());
    }
    // ==================== 诊断代码结束 ====================
    m_imu_cache.clear();
    m_imu_cache.push_back(m_last_imu);
    m_imu_cache.insert(m_imu_cache.end(), package.imus.begin(), package.imus.end());

    // const double imu_time_begin = m_imu_cache.front().time;
    const double imu_time_end = m_imu_cache.back().time;

    const double cloud_time_begin = package.cloud_start_time;
    const double propagate_time_end = package.lidar_end ? package.cloud_end_time : package.image_time;

    if (m_pushed)
    {
        m_poses_cache.clear();
        m_poses_cache.emplace_back(0.0, m_last_acc, m_last_gyro, m_kf->x().v, m_kf->x().t_wi, m_kf->x().r_wi);
        m_pushed = false;
    }

    V3D acc_val, gyro_val;
    double dt = 0.0;
    Input inp;
    inp.acc = m_imu_cache.back().acc;
    inp.gyro = m_imu_cache.back().gyro;
    for (auto it_imu = m_imu_cache.begin(); it_imu < (m_imu_cache.end() - 1); it_imu++)
    {
        IMUData &head = *it_imu;
        IMUData &tail = *(it_imu + 1);
        if (tail.time < m_last_propagate_end_time)
            continue;
        gyro_val = 0.5 * (head.gyro + tail.gyro);
        acc_val = 0.5 * (head.acc + tail.acc);

        if (head.time < m_last_propagate_end_time)
            dt = tail.time - m_last_propagate_end_time;
        else
            dt = tail.time - head.time;

        inp.acc = acc_val;
        inp.gyro = gyro_val;
        m_kf->predict(inp, dt, m_Q);

        m_last_gyro = gyro_val - m_kf->x().bg;
        m_last_acc = m_kf->x().r_wi * (acc_val - m_kf->x().ba) + m_kf->x().g;
        double offset = tail.time - cloud_time_begin;
        m_poses_cache.emplace_back(offset, m_last_acc, m_last_gyro, m_kf->x().v, m_kf->x().t_wi, m_kf->x().r_wi);
    }

    dt = propagate_time_end - imu_time_end;
    m_kf->predict(inp, dt, m_Q);
    m_last_imu = m_imu_cache.back();
    m_last_propagate_end_time = propagate_time_end;

    if (package.lidar_end)
    {
        M3D cur_r_wi = m_kf->x().r_wi;
        V3D cur_t_wi = m_kf->x().t_wi;
        M3D cur_r_il = m_kf->x().r_il;
        V3D cur_t_il = m_kf->x().t_il;
        auto it_pcl = package.cloud->points.end() - 1;

        for (auto it_kp = m_poses_cache.end() - 1; it_kp != m_poses_cache.begin(); it_kp--)
        {
            auto head = it_kp - 1;
            auto tail = it_kp;

            M3D imu_r_wi = head->rot;
            V3D imu_t_wi = head->trans;
            V3D imu_vel = head->vel;
            V3D imu_acc = tail->acc;
            V3D imu_gyro = tail->gyro;

            for (; it_pcl->curvature / double(1000) > head->offset; it_pcl--)
            {
                dt = it_pcl->curvature / double(1000) - head->offset;
                V3D point(it_pcl->x, it_pcl->y, it_pcl->z);
                M3D point_rot = imu_r_wi * Sophus::SO3d::exp(imu_gyro * dt).matrix();
                V3D point_pos = imu_t_wi + imu_vel * dt + 0.5 * imu_acc * dt * dt;
                V3D p_compensate = cur_r_il.transpose() * (cur_r_wi.transpose() * (point_rot * (cur_r_il * point + cur_t_il) + point_pos - cur_t_wi) - cur_t_il);
                it_pcl->x = p_compensate(0);
                it_pcl->y = p_compensate(1);
                it_pcl->z = p_compensate(2);
                if (it_pcl == package.cloud->points.begin())
                    break;
            }
        }
        m_pushed = true;
    }
}