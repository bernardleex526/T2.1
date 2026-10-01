// Behavioural regression for the LiDAR frontend converters (src/utils.cpp):
//   * which INPUT indices survive the tag / line / range / stride rules (livox2PCL);
//   * which input indices the PointCloud2 decimation keeps, and at which per-point time
//     origin those points are rebased (pcl2_to_PCL, incl. the C2.3 sampling phase).
// Only the returned clouds are inspected - no source text, no implementation detail.
#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

#include <livox_ros_driver2/msg/custom_msg.hpp>

#include "utils.h"

namespace {

using Cloud = pcl::PointCloud<pcl::PointXYZINormal>;
using Wrapped = Cloud::Ptr;

// Index markers: every synthetic point carries its input index in `intensity`, so the retained
// set is read back as an index list.
std::vector<int> intensities(const Cloud &c)
{
    std::vector<int> v;
    v.reserve(c.size());
    for (const auto &p : c)
        v.push_back(static_cast<int>(std::lround(p.intensity)));
    return v;
}

std::vector<float> curvatures(const Cloud &c)
{
    std::vector<float> v;
    v.reserve(c.size());
    for (const auto &p : c)
        v.push_back(p.curvature);
    return v;
}

// CustomMsg: line = i % 6, tag 0x00/0x10 = valid returns, 0x20/0x30 = invalid return; x carries
// the range pattern (0 => below lidar_min_range, 100 => above lidar_max_range, else 1 m).
livox_ros_driver2::msg::CustomMsg::SharedPtr makeLivox(std::size_t n)
{
    auto msg = std::make_shared<livox_ros_driver2::msg::CustomMsg>();
    msg->point_num = static_cast<uint32_t>(n);
    msg->points.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
        auto &p = msg->points[i];
        p.line = static_cast<uint8_t>(i % 6);
        p.tag = (i % 2 == 0) ? 0x00 : 0x10;           // valid returns
        if (i == 3 || i == 7)
            p.tag = 0x20;                             // invalid return
        if (i == 5)
            p.tag = 0x30;                             // invalid return
        p.x = (i == 2) ? 0.0f : ((i == 9) ? 100.0f : 1.0f);
        p.y = 0.0f;
        p.z = 0.0f;
        p.reflectivity = static_cast<float>(i);        // input-index marker
        p.offset_time = static_cast<uint32_t>(i) * 1000;  // 1000 ns per index -> 0.001 ms/idx
    }
    return msg;
}

// PointCloud2: x,y,z,intensity,time as FLOAT32 (point_step 20).  `time[i]` is the per-point
// field value; ranges: index 3 is a zero point, index 10 is 100 m out, everything else 1 m.
sensor_msgs::msg::PointCloud2::SharedPtr makePcl2(std::size_t n, bool with_xyz = true)
{
    using PF = sensor_msgs::msg::PointField;
    auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    msg->height = 1;
    msg->width = static_cast<uint32_t>(n);
    msg->point_step = 20;
    msg->row_step = msg->point_step * msg->width;
    msg->is_dense = true;
    auto add_field = [&](const char *name, uint32_t off, uint8_t dt) {
        PF f;
        f.name = name;
        f.offset = off;
        f.datatype = dt;
        f.count = 1;
        msg->fields.push_back(f);
    };
    if (!with_xyz) {
        add_field("a", 0, PF::FLOAT32);
        add_field("b", 4, PF::FLOAT32);
    } else {
        add_field("x", 0, PF::FLOAT32);
        add_field("y", 4, PF::FLOAT32);
        add_field("z", 8, PF::FLOAT32);
        add_field("intensity", 12, PF::FLOAT32);
        add_field("time", 16, PF::FLOAT32);
    }
    msg->data.assign(static_cast<std::size_t>(msg->point_step) * msg->width, 0);
    if (!with_xyz)
        return msg;
    for (std::size_t i = 0; i < n; ++i) {
        float *p = reinterpret_cast<float *>(msg->data.data() + i * msg->point_step);
        p[0] = (i == 3) ? 0.0f : ((i == 10) ? 100.0f : 1.0f);
        p[1] = 0.0f;
        p[2] = 0.0f;
        p[3] = static_cast<float>(i);            // input-index marker
        p[4] = static_cast<float>(i) * 0.01f;    // per-point time, seconds
    }
    return msg;
}

}  // namespace

// ---- livox2PCL: tag / line / range selection ------------------------------------------------
// 12 points, stride 2 (raw index), max_line 4, range [0.5, 20]:
//   kept: 0, 6, 8   (2 = below min range, 4/10 = line >= max_line, 9 = above max range,
//                     3/5/7 = invalid return tag)
// and index 1 - a valid, in-range point - is NOT kept, because the stride is applied to the RAW
// index before the tag/line/range rules (the historical, upstream-shaped order).
TEST(UtilsPreprocess, LivoxTagLineRangeSelection)
{
    auto msg = makeLivox(12);
    Wrapped c = Utils::livox2PCL(msg, 2, 0.5, 20.0, 4);
    EXPECT_EQ(intensities(*c), (std::vector<int>{0, 6, 8}));
    // curvature = offset_time / 1e6 (ms), independent of any phase concept
    EXPECT_NEAR(c->points[0].curvature, 0.0f, 1e-6f);
    EXPECT_NEAR(c->points[1].curvature, 0.006f, 1e-6f);  // index 6
    EXPECT_NEAR(c->points[2].curvature, 0.008f, 1e-6f);  // index 8
}

// A stride of 1 keeps every point that passes the tag/line/range rules.
TEST(UtilsPreprocess, LivoxStrideOneKeepsEveryValidPoint)
{
    auto msg = makeLivox(12);
    Wrapped c = Utils::livox2PCL(msg, 1, 0.5, 20.0, 4);
    EXPECT_EQ(intensities(*c), (std::vector<int>{0, 1, 6, 8}));
}

// ---- pcl2_to_PCL: decimation phase and per-point time origin ---------------------------------
// 13 points; index 3 is a zero point (range 0 < min), index 10 is out of range (100 m).
// Stride 1 keeps 0,1,2,4,5,6,7,8,9,11,12 in order, rebased on the first RETAINED point (index 0).
TEST(UtilsPreprocess, Pcl2StrideOneRangeFilterAndTimeOrigin)
{
    auto msg = makePcl2(13);
    Wrapped c = Utils::pcl2_to_PCL(msg, 1, 0.5, 20.0, "time", 1.0);
    EXPECT_EQ(intensities(*c), (std::vector<int>{0, 1, 2, 4, 5, 6, 7, 8, 9, 11, 12}));
    auto k = curvatures(*c);
    EXPECT_NEAR(k[0], 0.0f, 1e-6f);    // t0 = time of the first retained point
    EXPECT_NEAR(k[1], 10.0f, 1e-4f);   // index 1: 0.01 s later -> 10 ms
    EXPECT_NEAR(k[10], 120.0f, 1e-4f); // index 12: 0.12 s later -> 120 ms
}

// Phase 0 keeps the even input indices (the historical/upstream `i % filter_num == 0`).
TEST(UtilsPreprocess, Pcl2PhaseZeroKeepsEvenIndices)
{
    auto msg = makePcl2(13);
    Wrapped c = Utils::pcl2_to_PCL(msg, 2, 0.5, 20.0, "time", 1.0, 0);
    EXPECT_EQ(intensities(*c), (std::vector<int>{0, 2, 4, 6, 8, 12}));
    EXPECT_NEAR(curvatures(*c)[0], 0.0f, 1e-6f);
    EXPECT_NEAR(curvatures(*c)[5], 120.0f, 1e-4f);  // index 12
}

// Phase 1 keeps the odd input indices AND moves the time origin: the first retained point is
// index 1 (t = 0.01 s), so every curvature is rebased by -10 ms.
TEST(UtilsPreprocess, Pcl2PhaseOneKeepsOddIndicesAndMovesTimeOrigin)
{
    auto msg = makePcl2(13);
    Wrapped c = Utils::pcl2_to_PCL(msg, 2, 0.5, 20.0, "time", 1.0, 1);
    EXPECT_EQ(intensities(*c), (std::vector<int>{1, 5, 7, 9, 11}));  // 3 is the zero point
    auto k = curvatures(*c);
    EXPECT_NEAR(k[0], 0.0f, 1e-6f);    // index 1 is the origin now
    EXPECT_NEAR(k[1], 40.0f, 1e-4f);   // index 5: (0.05 - 0.01) s
    EXPECT_NEAR(k[4], 100.0f, 1e-4f);  // index 11
}

// Stride 3, phase 2 keeps exactly the input indices i == 2 (mod 3), again rebased on index 2.
TEST(UtilsPreprocess, Pcl2PhaseTwoWithStrideThree)
{
    auto msg = makePcl2(13);
    Wrapped c = Utils::pcl2_to_PCL(msg, 3, 0.5, 20.0, "time", 1.0, 2);
    EXPECT_EQ(intensities(*c), (std::vector<int>{2, 5, 8, 11}));
    auto k = curvatures(*c);
    EXPECT_NEAR(k[0], 0.0f, 1e-6f);
    EXPECT_NEAR(k[3], 90.0f, 1e-4f);  // index 11: (0.11 - 0.02) s
}

// The decimation runs on the RAW index, before the range filter: index 1 is in range but is
// dropped by the stride, and the out-of-range index 10 does not shift the kept indices.
TEST(UtilsPreprocess, Pcl2DecimatesRawIndicesBeforeRangeFilter)
{
    auto msg = makePcl2(13);
    Wrapped even = Utils::pcl2_to_PCL(msg, 2, 0.5, 20.0, "time", 1.0, 0);
    EXPECT_EQ(intensities(*even), (std::vector<int>{0, 2, 4, 6, 8, 12}));
    // the 0.5 m minimum also removes the zero points of phase 1 (index 3) but nothing else
    Wrapped odd = Utils::pcl2_to_PCL(msg, 2, 0.5, 20.0, "time", 1.0, 1);
    EXPECT_EQ(intensities(*odd), (std::vector<int>{1, 5, 7, 9, 11}));
}

// The production default (no phase argument) must be byte-identical to phase 0.
TEST(UtilsPreprocess, Pcl2DefaultPhaseIsZero)
{
    for (int stride : {1, 2, 3, 4}) {
        auto msg = makePcl2(13);
        Wrapped dflt = Utils::pcl2_to_PCL(msg, stride, 0.5, 20.0, "time", 1.0);
        Wrapped zero = Utils::pcl2_to_PCL(msg, stride, 0.5, 20.0, "time", 1.0, 0);
        EXPECT_EQ(intensities(*dflt), intensities(*zero)) << "stride " << stride;
        EXPECT_EQ(curvatures(*dflt).size(), curvatures(*zero).size()) << "stride " << stride;
        for (std::size_t i = 0; i < dflt->size(); ++i)
            EXPECT_FLOAT_EQ(dflt->points[i].curvature, zero->points[i].curvature) << "stride " << stride;
    }
}

// A phase outside [0, stride) is folded, and a degenerate filter_num behaves as stride 1.
TEST(UtilsPreprocess, Pcl2PhaseFoldingAndDegenerateStride)
{
    auto msg = makePcl2(13);
    Wrapped p5 = Utils::pcl2_to_PCL(msg, 2, 0.5, 20.0, "time", 1.0, 5);
    Wrapped p1 = Utils::pcl2_to_PCL(msg, 2, 0.5, 20.0, "time", 1.0, 1);
    EXPECT_EQ(intensities(*p5), intensities(*p1));
    Wrapped p3 = Utils::pcl2_to_PCL(msg, 3, 0.5, 20.0, "time", 1.0, 3);
    EXPECT_EQ(intensities(*p3), intensities(*Utils::pcl2_to_PCL(msg, 3, 0.5, 20.0, "time", 1.0, 0)));
    Wrapped neg = Utils::pcl2_to_PCL(msg, 3, 0.5, 20.0, "time", 1.0, -1);
    EXPECT_EQ(intensities(*neg), intensities(*Utils::pcl2_to_PCL(msg, 3, 0.5, 20.0, "time", 1.0, 2)));
    Wrapped zero_stride = Utils::pcl2_to_PCL(msg, 0, 0.5, 20.0, "time", 1.0, 0);
    EXPECT_EQ(intensities(*zero_stride), intensities(*Utils::pcl2_to_PCL(msg, 1, 0.5, 20.0, "time", 1.0, 0)));
}

// A message without xyz fields cannot be converted: empty cloud, no crash.
TEST(UtilsPreprocess, Pcl2WithoutXyzFieldsIsEmpty)
{
    auto msg = makePcl2(13, /*with_xyz=*/false);
    Wrapped c = Utils::pcl2_to_PCL(msg, 2, 0.5, 20.0, "time", 1.0, 1);
    EXPECT_TRUE(c->empty());
}

// No per-point time field configured: curvature stays 0 (no in-scan motion compensation).
TEST(UtilsPreprocess, Pcl2WithoutTimeFieldKeepsZeroCurvature)
{
    auto msg = makePcl2(13);
    Wrapped c = Utils::pcl2_to_PCL(msg, 2, 0.5, 20.0, "", 1.0, 1);
    EXPECT_EQ(intensities(*c), (std::vector<int>{1, 5, 7, 9, 11}));
    for (const auto &p : *c)
        EXPECT_FLOAT_EQ(p.curvature, 0.0f);
}
