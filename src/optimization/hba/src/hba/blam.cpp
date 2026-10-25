#include "blam.h"

#include <map>
#include <vector>
VoxelKey VoxelKey::index(double x, double y, double z, double resolution, double bias)
{
    V3D point(x, y, z);
    V3D idx = (point / resolution + V3D(bias, bias, bias)).array().floor();
    return VoxelKey(static_cast<int64_t>(idx(0)), static_cast<int64_t>(idx(1)), static_cast<int64_t>(idx(2)));
}
OctoTree::OctoTree(int layer, int max_layer, int min_point_num, double plane_thresh, double quater_len, const V3D &center)
    : m_layer(layer), m_max_layer(max_layer), m_min_point_num(min_point_num), m_plane_thresh(plane_thresh), m_quater_len(quater_len), m_is_valid(false), m_is_plane(false), m_center(center)
{
    m_leaves.resize(8, nullptr);
}
void OctoTree::insert(const PointType &point)
{
    m_points.push_back(point);
}
void OctoTree::buildPlanes()
{
    if (static_cast<int>(m_points.size()) < m_min_point_num)
    {
        m_is_plane = false;
        m_is_valid = false;
        return;
    }

    V3D mean = V3D::Zero();
    M3D cov = M3D::Zero();
    for (PointType &point : m_points)
    {
        V3D p_vec(point.x, point.y, point.z);
        mean += p_vec;
        cov += p_vec * p_vec.transpose();
    }
    mean /= static_cast<double>(m_points.size());
    cov = cov / static_cast<double>(m_points.size()) - mean * mean.transpose();
    Eigen::SelfAdjointEigenSolver<M3D> eigensolver(cov);
    m_mean = mean;
    m_eigen_val = eigensolver.eigenvalues();
    m_eigen_vec = eigensolver.eigenvectors();
    if (m_eigen_val[0] < m_plane_thresh)
    {
        m_mean = mean;
        m_is_plane = true;
        m_is_valid = true;
        doMergePoints();
        return;
    }
    m_is_plane = false;
    m_is_valid = splitPlanes();
}
void OctoTree::doMergePoints()
{
    assert(m_is_valid && m_is_plane);
    std::unordered_map<uint32_t, PointType> id_point_map;
    for (PointType &point : m_points)
    {
        uint32_t id = point.id;
        if (id_point_map.find(id) == id_point_map.end())
        {
            id_point_map[id] = point;
            id_point_map[id].time = 1.0;
        }
        else
        {
            id_point_map[id].x += point.x;
            id_point_map[id].y += point.y;
            id_point_map[id].z += point.z;
            id_point_map[id].intensity += point.intensity;
            id_point_map[id].lx += point.lx;
            id_point_map[id].ly += point.ly;
            id_point_map[id].lz += point.lz;
            id_point_map[id].time += 1.0;
        }
    }

    PointVec().swap(m_merged_points);
    for (auto &iter : id_point_map)
    {
        PointType &point = iter.second;
        point.x /= point.time;
        point.y /= point.time;
        point.z /= point.time;
        point.intensity /= point.time;
        point.lx /= point.time;
        point.ly /= point.time;
        point.lz /= point.time;
        point.time = 1.0;
        m_merged_points.push_back(point);
    }
}
bool OctoTree::splitPlanes()
{
    if (m_layer >= m_max_layer - 1)
        return false;
    for (PointType &point : m_points)
    {
        int xyz[3] = {0, 0, 0};
        if (point.x > m_center.x())
            xyz[0] = 1;
        if (point.y > m_center.y())
            xyz[1] = 1;
        if (point.z > m_center.z())
            xyz[2] = 1;

        int leaf_num = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
        if (m_leaves[leaf_num] == nullptr)
        {
            V3D shift((2 * xyz[0] - 1) * m_quater_len, (2 * xyz[1] - 1) * m_quater_len, (2 * xyz[2] - 1) * m_quater_len);
            V3D center = m_center + shift;
            m_leaves[leaf_num].reset(new OctoTree(m_layer + 1, m_max_layer, m_min_point_num, m_plane_thresh, m_quater_len / 2.0, center));
        }
        m_leaves[leaf_num]->insert(point);
    }

    PointVec().swap(m_points);
    PointVec().swap(m_merged_points);

    bool has_sub_plane = false;
    for (auto &leaf : m_leaves)
    {
        if (leaf != nullptr)
        {
            leaf->buildPlanes();
            if (leaf->isValid())
                has_sub_plane = true;
        }
    }
    return has_sub_plane;
}
void OctoTree::getPlanes(Vec<OctoTree *> &planes)
{
    if (m_is_plane)
    {
        planes.push_back(this);
        return;
    }
    if (!m_is_valid)
        return;

    for (auto &leaf : m_leaves)
    {
        if (leaf != nullptr)
            leaf->getPlanes(planes);
    }
}
int OctoTree::planeCount()
{
    if (m_is_plane)
        return 1;

    if (!m_is_valid)
        return 0;

    int count = 0;
    for (auto &leaf : m_leaves)
    {
        if (leaf != nullptr)
            count += leaf->planeCount();
    }
    return count;
}
double OctoTree::updateByPose(const Vec<Pose> &poses)
{
    assert(m_is_valid && m_is_plane);
    for (PointType &point : m_merged_points)
    {
        V3D p_vec(point.lx, point.ly, point.lz);
        const Pose &pose = poses[point.id];
        p_vec = pose.r * p_vec + pose.t;
        point.x = p_vec(0);
        point.y = p_vec(1);
        point.z = p_vec(2);
    }

    V3D mean = V3D::Zero();
    M3D cov = M3D::Zero();
    for (PointType &point : m_points)
    {
        V3D p_vec(point.lx, point.ly, point.lz);
        const Pose &pose = poses[point.id];
        p_vec = pose.r * p_vec + pose.t;
        point.x = p_vec(0);
        point.y = p_vec(1);
        point.z = p_vec(2);
        mean += p_vec;
        cov += p_vec * p_vec.transpose();
    }
    mean /= static_cast<double>(m_points.size());
    cov = cov / static_cast<double>(m_points.size()) - mean * mean.transpose();
    Eigen::SelfAdjointEigenSolver<M3D> eigensolver(cov);
    m_mean = mean;
    m_eigen_val = eigensolver.eigenvalues();
    m_eigen_vec = eigensolver.eigenvectors();
    return m_eigen_val[0];
}
double OctoTree::evalByPose(const Vec<Pose> &poses)
{
    assert(m_is_valid && m_is_plane);
    V3D mean = V3D::Zero();
    M3D cov = M3D::Zero();
    for (PointType &point : m_points)
    {
        V3D p_vec(point.lx, point.ly, point.lz);
        const Pose &pose = poses[point.id];
        p_vec = pose.r * p_vec + pose.t;
        point.x = p_vec(0);
        point.y = p_vec(1);
        point.z = p_vec(2);
        mean += p_vec;
        cov += p_vec * p_vec.transpose();
    }
    mean /= static_cast<double>(m_points.size());
    cov = cov / static_cast<double>(m_points.size()) - mean * mean.transpose();
    Eigen::SelfAdjointEigenSolver<M3D> eigensolver(cov);
    return eigensolver.eigenvalues()[0];
}
M3D OctoTree::fp(const V3D &p)
{
    M3D ret = M3D::Zero();
    ret.row(1) = (p - m_mean).transpose() * (m_eigen_vec.col(1) * m_eigen_vec.col(0).transpose() + m_eigen_vec.col(0) * m_eigen_vec.col(1).transpose()) / static_cast<double>(m_points.size()) / (m_eigen_val(0) - m_eigen_val(1));
    ret.row(2) = (p - m_mean).transpose() * (m_eigen_vec.col(2) * m_eigen_vec.col(0).transpose() + m_eigen_vec.col(0) * m_eigen_vec.col(2).transpose()) / static_cast<double>(m_points.size()) / (m_eigen_val(0) - m_eigen_val(2));
    return ret;
}
V3D OctoTree::dp(const V3D &p)
{
    return 2.0 / static_cast<double>(m_points.size()) * ((p - m_mean).transpose() * m_eigen_vec.col(0) * m_eigen_vec.col(0).transpose()).transpose();
}
M3D OctoTree::dp2(const V3D &p1, const V3D &p2, bool equal)
{
    double n = static_cast<double>(m_points.size());
    V3D u0 = m_eigen_vec.col(0);
    if (equal)
    {
        return 2.0 / n * ((n - 1) / n * u0 * u0.transpose() + u0 * (p1 - m_mean).transpose() * m_eigen_vec * fp(p2) + m_eigen_vec * fp(p2) * (u0.transpose() * (p1 - m_mean)));
    }
    else
    {
        return 2.0 / n * (-1.0 / n * u0 * u0.transpose() + u0 * (p1 - m_mean).transpose() * m_eigen_vec * fp(p2) + m_eigen_vec * fp(p2) * (u0.transpose() * (p1 - m_mean)));
    }
}
BLAM::BLAM(const BLAMConfig &config) : m_config(config)
{
    Vec<Pose>().swap(m_poses);
    Vec<pcl::PointCloud<pcl::PointXYZI>::Ptr>().swap(m_clouds);
}
void BLAM::insert(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, const Pose &pose)
{
    m_poses.push_back(pose);
    m_clouds.push_back(cloud);
}

void BLAM::addPoint(const PointType &point)
{
    VoxelKey loc = VoxelKey::index(point.x, point.y, point.z, m_config.voxel_size, 0.0);
    if (m_voxel_map.find(loc) == m_voxel_map.end())
    {
        V3D center = V3D((0.5 + loc.x) * m_config.voxel_size, (0.5 + loc.y) * m_config.voxel_size, (0.5 + loc.z) * m_config.voxel_size);
        double quater_len = m_config.voxel_size / 4.0;
        m_voxel_map[loc] = std::make_shared<OctoTree>(0, m_config.max_layer, m_config.min_point_num, m_config.plane_thresh, quater_len, center);
    }
    m_voxel_map[loc]->insert(point);
}
void BLAM::buildVoxels()
{

    Vec<OctoTree *>().swap(m_planes);
    VoxelMap().swap(m_voxel_map);
    for (size_t i = 0; i < m_clouds.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = m_clouds[i];
        const Pose &pose = m_poses[i];
        for (pcl::PointXYZI &point : cloud->points)
        {
            PointType p;
            V3D p_vec(point.x, point.y, point.z);
            p_vec = pose.r * p_vec + pose.t;
            p.x = static_cast<float>(p_vec.x());
            p.y = static_cast<float>(p_vec.y());
            p.z = static_cast<float>(p_vec.z());
            p.lx = point.x;
            p.ly = point.y;
            p.lz = point.z;
            p.intensity = point.intensity;
            p.id = static_cast<uint32_t>(i);
            p.time = 1.0;
            addPoint(p);
        }
    }

    for (auto &iter : m_voxel_map)
    {
        iter.second->buildPlanes();
        iter.second->getPlanes(m_planes);
    }
}
void BLAM::updateJaccAndHess()
{
    // Exact gradient and Hessian of f = sum over planes of lambda_0(C), the smallest eigenvalue of
    //     C = 1/N sum_j p_j p_j^T - mu mu^T,   p_j = R_i q_j + t_i  (q_j: point in its pose's frame),
    // w.r.t. the increment of plusDelta (R <- R Exp(phi), t <- t + tau; x_i = [phi_i; tau_i]).
    // Eigenvalue perturbation (u_k, lambda_k: eigenpairs of C, u = u_0):
    //     d_a lambda    = u^T C_a u
    //     d_ab lambda   = u^T C_ab u + 2 sum_{k=1,2} (u_k^T C_a u)(u_k^T C_b u) / (lambda_0 - lambda_k)
    //     u^T C_a u     = 2/N sum_j (u.p_ja)(u.(p_j - mu))
    //     u^T C_ab u    = 2/N sum_j [(u.p_jab)(u.(p_j - mu)) + (u.p_ja)(u.p_jb)] - 2 (u.mu_a)(u.mu_b)
    //     u_k^T C_a u   = 1/N sum_j [(u_k.p_ja)(u.(p_j - mu)) + (u.p_ja)(u_k.(p_j - mu))]
    // with p_j,phi = -R hat(q_j), p_j,tau = I, and p_j,phi_a phi_b = 1/2 R (e_a^ e_b^ + e_b^ e_a^) q_j.
    // Every sum is at most quadratic in q, so per (plane, pose) it only needs the point cluster
    // n, S1 = sum q, S2 = sum q q^T.  With w = R^T u, w_k = R^T u_k, c = u.(t - mu), c_k = u_k.(t - mu):
    //     sum_j (u.(p_j-mu)) [p_j,phi^T u ; u]  = [-hat(w)(S2 w + c S1) ; (w.S1 + n c) u]      (gradient)
    //     sum_j [p_j,x^T u][p_j,x^T u]^T        = [[hat(w) S2 hat(w)^T, -hat(w) S1 u^T], [., n u u^T]]
    //     sum_j (u.(p_j-mu)) u^T p_j,phi phi    = 1/2 (S2 w w^T + w w^T S2) + c/2 (S1 w^T + w S1^T)
    //                                             - (w^T S2 w + c w.S1) I
    //     N (u.mu_x)                            = Q = [-hat(w) S1 ; n u]
    //     N (u_k^T C_x u)                       = [-hat(w_k)(S2 w + c S1) - hat(w)(S2 w_k + c_k S1) ;
    //                                              (w.S1 + n c) u_k + (w_k.S1 + n c_k) u]
    // (This replaces a "merged point" form that averaged each pose's points: it dropped the per-pose
    //  count, mapped points onto the wrong poses through a column-index slip in D_i, and could not see
    //  a pose's own patch tilting about its centroid.)
    m_H.setZero();
    m_J.setZero();
    struct Cluster
    {
        double n = 0.0;
        V3D s1 = V3D::Zero();
        M3D s2 = M3D::Zero();
    };
    using V6D = Eigen::Matrix<double, 6, 1>;
    for (OctoTree *plane_ptr : m_planes)
    {
        const PointVec &points = plane_ptr->points();
        const double N = static_cast<double>(points.size());
        if (N < 1.0)
            continue;
        const V3D mu = plane_ptr->mean();
        const M3D U = plane_ptr->eigenVec();
        const V3D lam = plane_ptr->eigenVal();
        const V3D u = U.col(0);

        std::map<uint32_t, Cluster> clusters;
        for (const PointType &pt : points)
        {
            const V3D q(pt.lx, pt.ly, pt.lz);
            Cluster &c = clusters[pt.id];
            c.n += 1.0;
            c.s1 += q;
            c.s2 += q * q.transpose();
        }

        const size_t m = clusters.size();
        std::vector<uint32_t> ids;
        std::vector<V6D> Q(m), V1(m), V2(m);
        std::vector<M6D> A(m);
        size_t idx = 0;
        for (const auto &kv : clusters)
        {
            const Cluster &cl = kv.second;
            const Pose &pose = m_poses[kv.first];
            const V3D w = pose.r.transpose() * u;
            const double c = u.dot(pose.t - mu);
            const M3D hw = Sophus::SO3d::hat(w);
            const V3D s2w = cl.s2 * w;
            const V3D dsum_q = s2w + c * cl.s1;       // sum_j (u.(p_j - mu)) q_j
            const double dsum = w.dot(cl.s1) + cl.n * c; // sum_j (u.(p_j - mu))

            m_J.block<3, 1>(kv.first * 6, 0) += 2.0 / N * (-hw * dsum_q);
            m_J.block<3, 1>(kv.first * 6 + 3, 0) += 2.0 / N * dsum * u;

            M6D a = M6D::Zero();
            const M3D t1 = 0.5 * (s2w * w.transpose() + w * s2w.transpose()) +
                           0.5 * c * (cl.s1 * w.transpose() + w * cl.s1.transpose()) -
                           (w.dot(s2w) + c * w.dot(cl.s1)) * M3D::Identity();
            a.block<3, 3>(0, 0) = t1 + hw * cl.s2 * hw.transpose();
            a.block<3, 3>(0, 3) = -hw * cl.s1 * u.transpose();
            a.block<3, 3>(3, 0) = a.block<3, 3>(0, 3).transpose();
            a.block<3, 3>(3, 3) = cl.n * u * u.transpose();
            A[idx] = 2.0 / N * a;

            Q[idx].head<3>() = -hw * cl.s1;
            Q[idx].tail<3>() = cl.n * u;
            for (int k = 1; k <= 2; ++k)
            {
                const V3D uk = U.col(k);
                const V3D wk = pose.r.transpose() * uk;
                const double ck = uk.dot(pose.t - mu);
                V6D v;
                v.head<3>() = -Sophus::SO3d::hat(wk) * dsum_q - hw * (cl.s2 * wk + ck * cl.s1);
                v.tail<3>() = dsum * uk + (wk.dot(cl.s1) + cl.n * ck) * u;
                (k == 1 ? V1[idx] : V2[idx]) = v / N;
            }
            ids.push_back(kv.first);
            ++idx;
        }

        const double g1 = lam(0) - lam(1), g2 = lam(0) - lam(2);
        for (size_t i = 0; i < m; ++i)
            for (size_t l = 0; l < m; ++l)
            {
                M6D h = -2.0 / (N * N) * Q[i] * Q[l].transpose();
                if (std::abs(g1) > 1e-12)
                    h += 2.0 / g1 * V1[i] * V1[l].transpose();
                if (std::abs(g2) > 1e-12)
                    h += 2.0 / g2 * V2[i] * V2[l].transpose();
                if (i == l)
                    h += A[i];
                m_H.block<6, 6>(ids[i] * 6, ids[l] * 6) += h;
            }
    }
}
void BLAM::optimize()
{
    buildVoxels();
    Eigen::MatrixXd D;
    D.resize(m_poses.size() * 6, m_poses.size() * 6);
    m_H.resize(m_poses.size() * 6, m_poses.size() * 6);
    m_J.resize(m_poses.size() * 6);
    double residual = 0.0;
    bool build_hess = true;
    double u = 0.01, v = 2.0;

    for (size_t i = 0; i < m_config.max_iter; i++)
    {
        if (build_hess)
        {
            residual = updatePlanesByPoses(m_poses);
            updateJaccAndHess();
        }
        D = m_H.diagonal().asDiagonal();
        Eigen::MatrixXd Hess = m_H + u * D;
        Eigen::VectorXd delta = Hess.colPivHouseholderQr().solve(-m_J);
        Vec<Pose> temp_pose(m_poses.begin(), m_poses.end());
        plusDelta(temp_pose, delta);
        double residual_new = evalPlanesByPoses(temp_pose);
        double pho = (residual - residual_new) / (0.5 * (delta.transpose() * (u * D * delta - m_J))[0]);
        if (pho > 0)
        {
            build_hess = true;
            // std::cout << "ITER : " << i << " LM ITER UPDATE" << std::endl;
            m_poses = temp_pose;
            v = 2.0;
            double q = 1 - pow(2 * pho - 1, 3);
            u *= (q < 0.33333333 ? 0.33333333 : q);
        }
        else
        {
            build_hess = false;
            u *= v;
            v *= 2;
        }
        if (std::abs(residual - residual_new) < 1e-9)
            break;
    }
    // relativeInformation() reads the Hessian: build it at the FINAL poses (the last accepted step
    // moved them after the last build)
    updatePlanesByPoses(m_poses);
    updateJaccAndHess();
}

M6D BLAM::relativeInformation(size_t i, size_t j) const
{
    // m_H is in BLAM's increment delta_k = (phi_k right rotation, tau_k world translation).  Re-express
    // it in LEFT (world) perturbations zeta_k = [w_k; v_k], T_k <- Exp(zeta_k) T_k:
    //     phi_k = R_k^T w_k,   tau_k = v_k - hat(t_k) w_k     =>  delta_k = B_k zeta_k.
    // A common zeta moves the whole window rigidly and leaves every lambda_min unchanged, so in these
    // coordinates the quadratic form is a block Laplacian (zero block-row sums) and decomposes exactly
    // into pairwise terms:  z^T H z = sum_{a<b} (z_a - z_b)^T (-sym H_ab) (z_a - z_b).
    // Hence the pair (i, j) carries Lambda_z = -sym(B_i^T H_ij B_j) (the old code used +H_ij in the
    // delta coordinates: wrong sign, wrong frame, not symmetric).  GTSAM's BetweenFactor error e is the
    // right perturbation of T_i^-1 T_j, and z_j - z_i = Ad(T_j) e, so Lambda_e = Ad^T Lambda_z Ad with
    // Ad(T) = [[R, 0], [hat(t) R, R]] for the [rot; trans] ordering.  The approximate (merged-point)
    // Hessian is only nearly gauge invariant, so the result is projected onto the PD cone.
    auto B = [this](size_t k) {
        M6D b = M6D::Zero();
        b.block<3, 3>(0, 0) = m_poses[k].r.transpose();
        b.block<3, 3>(3, 0) = -Sophus::SO3d::hat(m_poses[k].t);
        b.block<3, 3>(3, 3) = M3D::Identity();
        return b;
    };
    const M6D h_ij = m_H.block<6, 6>(6 * i, 6 * j);
    const M6D hz = B(i).transpose() * h_ij * B(j);
    const M6D lz = -0.5 * (hz + hz.transpose());
    M6D ad = M6D::Zero();
    ad.block<3, 3>(0, 0) = m_poses[j].r;
    ad.block<3, 3>(3, 0) = Sophus::SO3d::hat(m_poses[j].t) * m_poses[j].r;
    ad.block<3, 3>(3, 3) = m_poses[j].r;
    M6D le = ad.transpose() * lz * ad;
    le = 0.5 * (le + le.transpose());
    Eigen::SelfAdjointEigenSolver<M6D> es(le);
    const double floor = 1e-6 * std::max(es.eigenvalues().maxCoeff(), 1e-12);
    const Eigen::Matrix<double, 6, 1> ev = es.eigenvalues().cwiseMax(floor);
    return es.eigenvectors() * ev.asDiagonal() * es.eigenvectors().transpose();
}

double BLAM::updatePlanesByPoses(const Vec<Pose> &poses)
{
    double residual = 0.0;
    for (size_t i = 0; i < m_planes.size(); i++)
    {
        OctoTree *plane = m_planes[i];
        residual += plane->updateByPose(poses);
    }
    return residual;
}

double BLAM::evalPlanesByPoses(const Vec<Pose> &poses)
{
    double residual = 0.0;
    for (size_t i = 0; i < m_planes.size(); i++)
    {
        OctoTree *plane = m_planes[i];
        residual += plane->evalByPose(poses);
    }
    return residual;
}

int BLAM::planeCount(bool with_sub_planes)
{
    int count = 0;
    for (auto &iter : m_voxel_map)
    {
        if (with_sub_planes)
            count += iter.second->planeCount();
        else if (iter.second->isPlane())
            count += 1;
    }
    return count;
}

void BLAM::plusDelta(Vec<Pose> &poses, const Eigen::VectorXd &x)
{
    assert(static_cast<size_t>(x.rows()) == poses.size() * 6);
    for (size_t i = 0; i < poses.size(); i++)
    {
        M3D &r = poses[i].r;
        V3D &t = poses[i].t;
        r = r * Sophus::SO3d::exp(x.segment<3>(i * 6)).matrix();
        t += x.segment<3>(i * 6 + 3);
    }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr BLAM::getLocalCloud()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr ret(new pcl::PointCloud<pcl::PointXYZI>);
    Pose &first_pose = m_poses[0];

    for (OctoTree *plane : m_planes)
    {
        for (const PointType &point : plane->points())
        {
            pcl::PointXYZI p;
            Pose &cur_pose = m_poses[point.id];
            M3D r_fc = first_pose.r.transpose() * cur_pose.r;
            V3D t_fc = first_pose.r.transpose() * (cur_pose.t - first_pose.t);
            V3D p_vec(point.lx, point.ly, point.lz);
            p_vec = r_fc * p_vec + t_fc;
            p.x = p_vec[0];
            p.y = p_vec[1];
            p.z = p_vec[2];
            p.intensity = point.intensity;
            ret->push_back(p);
        }
    }
    return ret;
}