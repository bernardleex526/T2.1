#include "commons.h"
#include <cmath>

// 平面拟合：质心 + 协方差最小特征值对应的特征向量(PCA)，out = (n, d)，|n| = 1，n·p + d = 0。
// 上游 FAST-LIO2 解 A*n' = -1(即 n'·p + 1 = 0)，该参数化无法表示过世界原点的平面(d = 0)：
// 原点 = IMU 初始化位置，与之等高的台面/台阶会被小邻域(5 点、约 10 cm)拟合成一个能通过
// 0.1 m 阈值的"竖直假平面"，给出错误的法向约束。PCA 与原点位置无关。
bool esti_plane(PointVec &points, const double &thresh, V4D &out)
{
    const size_t n = points.size();
    if (n < 3)
        return false;
    V3D centroid = V3D::Zero();
    for (const PointType &p : points)
        centroid += V3D(p.x, p.y, p.z);
    centroid /= static_cast<double>(n);
    M3D cov = M3D::Zero();
    for (const PointType &p : points)
    {
        const V3D d = V3D(p.x, p.y, p.z) - centroid;
        cov += d * d.transpose();
    }
    cov /= static_cast<double>(n);
    Eigen::SelfAdjointEigenSolver<M3D> es(cov);
    // 近共线(第二小特征值几乎为零)时法向在垂直于该直线的平面内任意，不构成平面观测
    if (es.info() != Eigen::Success || es.eigenvalues()(1) < 1e-8)
        return false;
    const V3D normal = es.eigenvectors().col(0);
    out[0] = normal(0);
    out[1] = normal(1);
    out[2] = normal(2);
    out[3] = -normal.dot(centroid);
    for (size_t j = 0; j < n; j++)
    {
        if (std::fabs(out(0) * points[j].x + out(1) * points[j].y + out(2) * points[j].z + out(3)) > thresh)
        {
            return false;
        }
    }
    return true;
}

float sq_dist(const PointType &p1, const PointType &p2)
{
    return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
}
