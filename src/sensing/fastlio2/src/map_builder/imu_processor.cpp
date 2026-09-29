#include "imu_processor.h"

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
    const bool use_static_window = m_config.imu_init_window_s > 0.0;
    if (!use_static_window)
    {
        if (m_imu_cache.size() < static_cast<size_t>(m_config.imu_init_num))
            return false;
        // legacy path: initialize from everything collected so far (see below via window == full cache)
    }
    else
    {
        const double span = m_imu_cache.back().time - m_imu_cache.front().time;
        if (span < m_config.imu_init_window_s && span < m_config.imu_init_max_wait_s)
            return false;
    }

    // Pick the analysis window: legacy = whole cache; static-window mode = the quietest
    // contiguous window of imu_init_window_s inside the cache (best = min per-axis gyro std).
    // Scoring all windows keeps the init general: whatever the platform (dog, handheld),
    // bg/gravity are estimated over a genuinely stationary stretch when one exists.
    size_t n = m_imu_cache.size();
    size_t i0 = 0, i1 = n; // [i0, i1)
    if (use_static_window)
    {
        const double w = m_config.imu_init_window_s;
        size_t wlen = 1;
        while (wlen < n && m_imu_cache[wlen].time - m_imu_cache[0].time < w)
            ++wlen;
        if (wlen >= n)
            wlen = n;
        double best_score = 1e18;
        for (size_t s = 0; s + wlen <= n; ++s)
        {
            V3D gm = V3D::Zero(), am = V3D::Zero(), gdev = V3D::Zero(), adev = V3D::Zero();
            for (size_t k = s; k < s + wlen; ++k)
            {
                gm += m_imu_cache[k].gyro;
                am += m_imu_cache[k].acc;
            }
            gm /= wlen; am /= wlen;
            for (size_t k = s; k < s + wlen; ++k)
            {
                gdev += (m_imu_cache[k].gyro - gm).cwiseAbs2();
                adev += (m_imu_cache[k].acc - am).cwiseAbs2();
            }
            gdev = (gdev / wlen).cwiseSqrt();
            adev = (adev / wlen).cwiseSqrt();
            double score = gdev.maxCoeff();
            if (score < best_score)
            {
                best_score = score;
                i0 = s;
                i1 = s + wlen;
            }
        }
        const double span = m_imu_cache.back().time - m_imu_cache.front().time;
        const bool waited_out = span >= m_config.imu_init_max_wait_s && !(span >= m_config.imu_init_window_s && best_score < m_config.imu_init_static_gyro_std);
        // Static verdict on the chosen window
        V3D gm = V3D::Zero(), gdev = V3D::Zero(), adev = V3D::Zero();
        {
            V3D am = V3D::Zero();
            for (size_t k = i0; k < i1; ++k)
            {
                gm += m_imu_cache[k].gyro;
                am += m_imu_cache[k].acc;
            }
            gm /= (i1 - i0);
            am /= (i1 - i0);
            for (size_t k = i0; k < i1; ++k)
            {
                gdev += (m_imu_cache[k].gyro - gm).cwiseAbs2();
                adev += (m_imu_cache[k].acc - am).cwiseAbs2();
            }
            gdev = (gdev / (i1 - i0)).cwiseSqrt();
            adev = (adev / (i1 - i0)).cwiseSqrt();
        }
        const bool is_static = gdev.maxCoeff() < m_config.imu_init_static_gyro_std &&
                               adev.maxCoeff() < m_config.imu_init_static_acc_dev;
        if (!is_static && !waited_out)
            return false; // keep waiting for a static window
        RCLCPP_WARN(m_logger, "IMU init window [%zu,%zu)/%zu gyro_std=%.5f acc_dev=%.4f static=%d waited_out=%d",
                    i0, i1, n, gdev.maxCoeff(), adev.maxCoeff(), (int)is_static, (int)waited_out);
    }

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
        // Only adopt the measured magnitude when the init window is genuinely static.
        V3D acc_dev = V3D::Zero();
        for (const auto &imu : m_imu_cache)
            acc_dev += (imu.acc - acc_mean).cwiseAbs2();
        acc_dev = (acc_dev / static_cast<double>(m_imu_cache.size())).cwiseSqrt();
        const double g_meas = acc_mean.norm();
        if (g_meas > 8.5 && g_meas < 10.5 && acc_dev.maxCoeff() < 0.3)
            State::gravity = g_meas;
        m_kf->x().r_wi = (Eigen::Quaterniond::FromTwoVectors((-acc_mean).normalized(), V3D(0.0, 0.0, -1.0)).matrix());
        m_kf->x().initGravityDir(V3D(0, 0, -1.0));
        RCLCPP_WARN(m_logger, "IMU init: g=%.4f (acc_dev_max=%.4f) bg=[%.5f %.5f %.5f] acc_mean=[%.4f %.4f %.4f]",
                    State::gravity, acc_dev.maxCoeff(),
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

    m_last_imu = m_imu_cache.back();
    m_last_propagate_end_time = package.cloud_end_time;
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