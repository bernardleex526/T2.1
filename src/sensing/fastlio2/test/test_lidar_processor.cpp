// LidarProcessor must accept a scan whose DOWNSAMPLED size exceeds any fixed capacity: an Airy at
// >=300 kHz with a 5 cm scan voxel gives >10k points per scan, which used to write past the end
// of the 10k-element per-point work buffers (segfault after the second scan).
#include <gtest/gtest.h>

#include <memory>

#include "map_builder/ieskf.h"
#include "map_builder/lidar_processor.h"

namespace {

// Points on the floor z=0 and two walls x=6 and y=6 of a corner, spaced `step` m apart.
CloudType::Ptr cornerCloud(double step, double extent)
{
    CloudType::Ptr c(new CloudType);
    for (double a = -extent; a <= extent; a += step)
        for (double b = -extent; b <= extent; b += step) {
            PointType p;
            p.x = a; p.y = b; p.z = 0.0; c->push_back(p);                       // floor
            p.x = 6.0; p.y = a; p.z = 0.5 * (b + extent) / extent * 3.0; c->push_back(p);  // wall x
            p.x = a; p.y = 6.0; c->push_back(p);                                 // wall y
        }
    return c;
}

TEST(LidarProcessorBuffers, DenseScanAboveTenThousandPointsIsProcessed)
{
    Config cfg;
    cfg.scan_resolution = 0.0;  // keep every point: the downsampled cloud is the raw cloud
    cfg.map_resolution = 0.1;
    cfg.near_search_num = 5;
    cfg.point_quality_thresh = 0.9;
    auto kf = std::make_shared<IESKF>();
    kf->setMaxIter(3);
    LidarProcessor lp(cfg, kf);

    PointVec map_pts;
    const CloudType::Ptr map_cloud = cornerCloud(0.1, 5.0);
    for (const auto &p : *map_cloud)
        map_pts.push_back(p);
    lp.initCloudMap(map_pts);

    SyncPackage pkg;
    pkg.cloud = cornerCloud(0.05, 4.0);
    ASSERT_GT(pkg.cloud->size(), 30000u);
    for (int scan = 0; scan < 3; ++scan)
        lp.process(pkg);

    // the true pose is the identity: a sane update leaves it there (and nothing crashed)
    EXPECT_LT(kf->x().t_wi.norm(), 0.02);
}

}  // namespace
