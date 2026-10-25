// BLAM (HBA's local bundle adjustment) regressions, checked against the EXACT cost
// f = sum over planes of lambda_min(cov of every plane point) and central differences:
//   * the gradient m_J used to be built from one averaged point per pose with a wrong column index
//     in D_i (poses in the second half of a plane got no gradient at all, cosine to the true
//     gradient 0.02) and without the per-pose point count;
//   * BLAM::optimize() therefore did not reduce the pose error;
//   * HBA used an off-diagonal Hessian block as a BetweenFactor information matrix (asymmetric,
//     indefinite, wrong sign).
#include <gtest/gtest.h>

#include <random>

#include "hba/blam.h"

namespace
{
struct Scene
{
    std::vector<Pose> truth;
    std::vector<Pose> init;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clouds;
};

// One planar patch per 1 m voxel (kept 0.25 m inside it), 40 points per pose and patch.
Scene makeScene(int n_poses, unsigned seed, double init_rot, double init_t)
{
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> U(-1, 1);
    struct Patch { V3D o, a, b; };
    const std::vector<Patch> patches = {
        {V3D(0.5, 0.5, 0.5), V3D(1, 0, 0), V3D(0, 1, 0)},
        {V3D(2.5, 0.5, 0.5), V3D(0, 1, 0), V3D(0, 0, 1)},
        {V3D(0.5, 3.5, 0.5), V3D(1, 0, 0), V3D(0, 0, 1)},
        {V3D(1.5, 1.5, 1.5), V3D(1, -1, 0).normalized(), V3D(1, 1, -1).normalized()},
        {V3D(-1.5, 0.5, 2.5), V3D(0.6, 0.8, 0), V3D(0, 0, 1)},
        {V3D(3.5, -1.5, 0.5), V3D(0, 0.8, 0.6), V3D(1, 0, 0)},
        {V3D(-0.5, -2.5, 1.5), V3D(1, 0, 0), V3D(0, 0.6, -0.8)}};
    Scene s;
    s.truth.resize(n_poses);
    for (int i = 0; i < n_poses; ++i)
    {
        s.truth[i].r = Sophus::SO3d::exp(V3D(U(rng), U(rng), U(rng)) * 0.2).matrix();
        s.truth[i].t = V3D(U(rng), U(rng), U(rng)) * 0.3;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        for (const Patch &pa : patches)
            for (int k = 0; k < 40; ++k)
            {
                const V3D pw = pa.o + 0.25 * U(rng) * pa.a + 0.25 * U(rng) * pa.b;
                const V3D pl = s.truth[i].r.transpose() * (pw - s.truth[i].t);
                pcl::PointXYZI q;
                q.x = pl.x(); q.y = pl.y(); q.z = pl.z(); q.intensity = 1;
                cloud->push_back(q);
            }
        s.clouds.push_back(cloud);
        Pose est;
        est.r = s.truth[i].r * Sophus::SO3d::exp(V3D(U(rng), U(rng), U(rng)) * init_rot).matrix();
        est.t = s.truth[i].t + V3D(U(rng), U(rng), U(rng)) * init_t;
        s.init.push_back(est);
    }
    return s;
}

BLAMConfig config()
{
    BLAMConfig cfg;
    cfg.voxel_size = 1.0;
    cfg.min_point_num = 10;
    cfg.plane_thresh = 0.01;
    return cfg;
}

// Worst relative-pose error w.r.t. pose 0 (BLAM has a free gauge).
void relErr(const std::vector<Pose> &truth, const Vec<Pose> &est, double *rot_deg, double *t_m)
{
    *rot_deg = 0.0;
    *t_m = 0.0;
    for (size_t i = 1; i < truth.size(); ++i)
    {
        const M3D rt = truth[0].r.transpose() * truth[i].r, re = est[0].r.transpose() * est[i].r;
        const V3D tt = truth[0].r.transpose() * (truth[i].t - truth[0].t);
        const V3D te = est[0].r.transpose() * (est[i].t - est[0].t);
        *rot_deg = std::max(*rot_deg, Sophus::SO3d(rt.transpose() * re).log().norm() * 180.0 / M_PI);
        *t_m = std::max(*t_m, (tt - te).norm());
    }
}

// BLAM over the scene, planes built, Hessian/gradient evaluated at the initial poses.
BLAM prepared(const Scene &sc)
{
    BLAM blam(config());
    for (size_t i = 0; i < sc.clouds.size(); ++i)
    {
        auto c = sc.clouds[i];
        blam.insert(c, sc.init[i]);
    }
    blam.buildVoxels();
    const int n6 = 6 * static_cast<int>(sc.clouds.size());
    blam.H().resize(n6, n6);
    blam.J().resize(n6);
    blam.updatePlanesByPoses(blam.poses());
    blam.updateJaccAndHess();
    return blam;
}
} // namespace

TEST(Blam, GradientMatchesCentralDifferencesOfTheExactCost)
{
    const Scene sc = makeScene(5, 3, 0.01, 0.01);
    BLAM blam = prepared(sc);
    ASSERT_GE(blam.planes().size(), 7u);
    const Eigen::VectorXd J = blam.J();
    const int n6 = static_cast<int>(J.size());
    Eigen::VectorXd jn(n6);
    const double e = 1e-6;
    for (int k = 0; k < n6; ++k)
    {
        Eigen::VectorXd d = Eigen::VectorXd::Zero(n6);
        d(k) = e;
        Vec<Pose> pp = blam.poses(), pm = blam.poses();
        BLAM::plusDelta(pp, d);
        BLAM::plusDelta(pm, -d);
        jn(k) = (blam.evalPlanesByPoses(pp) - blam.evalPlanesByPoses(pm)) / (2 * e);
    }
    EXPECT_LT((J - jn).norm() / jn.norm(), 1e-4);
    for (int i = 0; i < n6 / 6; ++i)
        EXPECT_GT(J.segment<6>(6 * i).norm(), 0.0) << "pose " << i << " got no gradient";
}

TEST(Blam, HessianMatchesSecondDifferencesOfTheExactCost)
{
    // second differences in ONE chart (x <+> (s e_a + t e_b)); differencing the gradient at moved
    // poses would mix in the chart change (BCH term) wherever the gradient is non-zero
    const Scene sc = makeScene(3, 5, 0.01, 0.01);
    BLAM blam = prepared(sc);
    const Eigen::MatrixXd H = blam.H();
    const int n6 = static_cast<int>(H.rows());
    const double e = 1e-4;
    auto f = [&](int a, double sa, int b, double sb) {
        Eigen::VectorXd d = Eigen::VectorXd::Zero(n6);
        d(a) += sa;
        d(b) += sb;
        Vec<Pose> p = blam.poses();
        BLAM::plusDelta(p, d);
        return blam.evalPlanesByPoses(p);
    };
    Eigen::MatrixXd hn(n6, n6);
    for (int a = 0; a < n6; ++a)
        for (int b = 0; b < n6; ++b)
            hn(a, b) = (f(a, e, b, e) - f(a, e, b, -e) - f(a, -e, b, e) + f(a, -e, b, -e)) / (4 * e * e);
    EXPECT_LT((H - hn).norm() / hn.norm(), 1e-3);
    EXPECT_LT((H - H.transpose()).norm(), 1e-9 * H.norm());
}

TEST(Blam, OptimizeRecoversTheRelativePoses)
{
    for (unsigned seed = 11; seed < 16; ++seed)
    {
        const Scene sc = makeScene(5, seed, 0.02, 0.03);
        BLAMConfig cfg = config();
        cfg.max_iter = 30; // rejected LM steps count too; 10 left one of these seeds at 0.48 deg
        BLAM blam(cfg);
        for (size_t i = 0; i < sc.clouds.size(); ++i)
        {
            auto c = sc.clouds[i];
            blam.insert(c, sc.init[i]);
        }
        double r0, t0, r1, t1;
        relErr(sc.truth, blam.poses(), &r0, &t0);
        blam.optimize();
        relErr(sc.truth, blam.poses(), &r1, &t1);
        EXPECT_LT(r1, 0.1 * r0) << "seed " << seed << ": " << r0 << " -> " << r1 << " deg";
        EXPECT_LT(t1, 0.1 * t0) << "seed " << seed << ": " << t0 << " -> " << t1 << " m";
    }
}

TEST(Blam, RelativeInformationIsSymmetricPositiveDefinite)
{
    const Scene sc = makeScene(5, 3, 0.002, 0.002);
    BLAM blam = prepared(sc);
    for (size_t i = 0; i + 1 < sc.clouds.size(); ++i)
    {
        const M6D info = blam.relativeInformation(i, i + 1);
        EXPECT_LT((info - info.transpose()).norm(), 1e-12 * info.norm());
        Eigen::SelfAdjointEigenSolver<M6D> es(info);
        EXPECT_GT(es.eigenvalues().minCoeff(), 0.0) << "pair " << i;
    }
}

// Two-pose window: the pairwise decomposition is exact, so e^T Lambda e must reproduce the
// curvature of the exact cost when T_1 is moved by a right perturbation e of T_0^-1 T_1.
TEST(Blam, TwoPoseRelativeInformationMatchesTheCostCurvature)
{
    Scene sc = makeScene(2, 21, 0.0, 0.0); // at the true poses: gradient ~ 0
    BLAM blam = prepared(sc);
    const M6D info = blam.relativeInformation(0, 1);
    std::mt19937 rng(4);
    std::normal_distribution<double> N(0.0, 1.0);
    for (int trial = 0; trial < 6; ++trial)
    {
        Eigen::Matrix<double, 6, 1> dir;
        for (int k = 0; k < 6; ++k)
            dir(k) = N(rng);
        dir.head<3>() *= 1e-2; // rad
        dir.tail<3>() *= 1e-2; // m
        const double h = 1e-2;
        auto cost = [&](double s) {
            Vec<Pose> p = blam.poses();
            const Eigen::Matrix<double, 6, 1> e = s * dir;
            // T_1 <- T_1 Exp(e) for e = [w; v] (first order: R Exp(w), t + R v)
            p[1].t = p[1].t + p[1].r * e.tail<3>();
            p[1].r = p[1].r * Sophus::SO3d::exp(e.head<3>()).matrix();
            return blam.evalPlanesByPoses(p);
        };
        const double curv = (cost(h) + cost(-h) - 2.0 * cost(0.0)) / (h * h); // = dir^T H_e dir
        const double model = dir.dot(info * dir);
        EXPECT_NEAR(model / curv, 1.0, 0.3) << "trial " << trial << " curvature " << curv << " model " << model;
    }
}
