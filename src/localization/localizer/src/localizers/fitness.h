#pragma once
// ---------------------------------------------------------------------------
// Nearest-neighbour fitness and inlier ratio of a scan against a map cloud.
//
// pcl::IterativeClosestPoint::getFitnessScore(max_range) compares max_range against the
// SQUARED nearest-neighbour distance (so 0.35 src units admits points out to ~0.59 m, which
// contradicts setMaxCorrespondenceDistance(0.35), also in metres) and returns the mean over
// whatever subset fell within range, with no inlier fraction.  A scan that only partially
// matches the map therefore scores just as well as a full match, which weakens the ungated
// first lock after `relocalize` (integration review N3).
//
// This computes both quantities explicitly, against a caller-supplied KD-tree so the target
// is indexed once (the map only changes in loadMap):
//   mean_sq_dist  - mean squared NN distance over the inliers [m^2], NaN-free
//   inlier_ratio  - inliers / source points, to gate partial matches
// ---------------------------------------------------------------------------
#include "commons.h"

#include <pcl/kdtree/kdtree_flann.h>

#include <limits>
#include <vector>

// Returns false only if either cloud is empty (nothing measurable).  Templated on the search
// object so the same index can be shared with the ICP itself (pcl::search::KdTree) or be a
// standalone FLANN tree; the search semantics are identical either way.
template <typename TreeT>
inline bool fitnessAndInliers(TreeT &target_tree,
                              const CloudType::Ptr &src, double corr_dist_m,
                              double *mean_sq_dist, double *inlier_ratio)
{
    if (!src || src->empty())
        return false;
    const double r2 = corr_dist_m * corr_dist_m;
    double sum = 0.0;
    size_t inliers = 0;
    std::vector<int> idx(1);
    std::vector<float> sq(1);
    for (const PointType &p : src->points)
    {
        if (target_tree.nearestKSearch(p, 1, idx, sq) < 1)
            continue;
        if (static_cast<double>(sq[0]) <= r2)
        {
            sum += static_cast<double>(sq[0]);
            ++inliers;
        }
    }
    *mean_sq_dist = inliers ? sum / static_cast<double>(inliers)
                            : std::numeric_limits<double>::max();
    *inlier_ratio = static_cast<double>(inliers) / static_cast<double>(src->size());
    return true;
}
