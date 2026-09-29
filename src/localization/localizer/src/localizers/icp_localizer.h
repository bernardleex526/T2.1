#pragma once
#include "commons.h"
#include <filesystem>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>

struct ICPConfig
{
    double refine_scan_resolution = 0.1;
    double refine_map_resolution = 0.1;
    double refine_score_thresh = 0.1;
    int refine_max_iteration = 10;

    double rough_scan_resolution = 0.25;
    double rough_map_resolution = 0.25;
    double rough_score_thresh = 0.2;
    int rough_max_iteration = 5;

    // Correspondence radius [m] used both by the ICP and by the fitness/inlier measurement,
    // and the minimum fraction of the scan that must fall within it.  The radius is in metres
    // (NOT the squared-distance argument PCL's getFitnessScore() takes - see fitness.h).
    double rough_max_corr_dist = 0.35;
    double refine_max_corr_dist = 0.15;
    double rough_min_inlier_ratio = 0.2;
    double refine_min_inlier_ratio = 0.2;
};

class ICPLocalizer
{
public:
    ICPLocalizer(const ICPConfig &config);
    
    bool loadMap(const std::string &path);
    
    void setInput(const CloudType::Ptr &cloud);

    bool align(M4F &guess);
    ICPConfig &config() { return m_config; }
    CloudType::Ptr roughMap() { return m_rough_tgt; }
    CloudType::Ptr refineMap() { return m_refine_tgt; }


private:
    ICPConfig m_config;
    pcl::VoxelGrid<PointType> m_voxel_filter;
    pcl::IterativeClosestPoint<PointType, PointType> m_refine_icp;
    pcl::IterativeClosestPoint<PointType, PointType> m_rough_icp;
    CloudType::Ptr m_refine_inp;
    CloudType::Ptr m_rough_inp;
    CloudType::Ptr m_refine_tgt;
    CloudType::Ptr m_rough_tgt;
    // Nearest-neighbour indices of the two target maps, built once in loadMap: the fitness and
    // inlier-ratio measurement in align() needs them on every update.
    pcl::KdTreeFLANN<PointType> m_refine_tree;
    pcl::KdTreeFLANN<PointType> m_rough_tree;
    std::string m_pcd_path;
};