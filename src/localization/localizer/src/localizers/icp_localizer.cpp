#include "icp_localizer.h"
#include "fitness.h"

ICPLocalizer::ICPLocalizer(const ICPConfig &config) : m_config(config)
{
    m_refine_inp.reset(new CloudType);
    m_refine_tgt.reset(new CloudType);
    m_rough_inp.reset(new CloudType);
    m_rough_tgt.reset(new CloudType);
}
bool ICPLocalizer::loadMap(const std::string &path)
{
    if (!std::filesystem::exists(path))
    {
        std::cerr << "Map file not found: " << path << std::endl;
        return false;
    }
    pcl::PCDReader reader;
    CloudType::Ptr cloud(new CloudType);
    reader.read(path, *cloud);
    if (m_config.refine_map_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.refine_map_resolution, m_config.refine_map_resolution, m_config.refine_map_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_refine_tgt);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *m_refine_tgt);
    }

    if (m_config.rough_map_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.rough_map_resolution, m_config.rough_map_resolution, m_config.rough_map_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_rough_tgt);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *m_rough_tgt);
    }

    m_rough_icp.setMaximumIterations(m_config.rough_max_iteration);
    m_rough_icp.setMaxCorrespondenceDistance(m_config.rough_max_corr_dist);
    m_rough_icp.setInputTarget(m_rough_tgt);

    m_refine_icp.setMaximumIterations(m_config.refine_max_iteration);
    m_refine_icp.setMaxCorrespondenceDistance(m_config.refine_max_corr_dist);
    m_refine_icp.setInputTarget(m_refine_tgt);

    // Index the targets once: the per-update fitness/inlier measurement needs a NN search, and
    // the maps only change here.
    m_rough_tree.setInputCloud(m_rough_tgt);
    m_refine_tree.setInputCloud(m_refine_tgt);
    return true;
}
void ICPLocalizer::setInput(const CloudType::Ptr &cloud)
{
    if (m_config.refine_scan_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.refine_scan_resolution, m_config.refine_scan_resolution, m_config.refine_scan_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_refine_inp);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *m_refine_inp);
    }

    if (m_config.rough_scan_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.rough_scan_resolution, m_config.rough_scan_resolution, m_config.rough_scan_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_rough_inp);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *m_rough_inp);
    }
}

bool ICPLocalizer::align(M4F &guess)
{
    CloudType::Ptr aligned_cloud(new CloudType);
    if (m_refine_tgt->size() == 0 || m_rough_tgt->size() == 0)
        return false;
    double score = 0.0, inlier_ratio = 0.0;
    m_rough_icp.setInputSource(m_rough_inp);
    m_rough_icp.align(*aligned_cloud, guess);
    if (!m_rough_icp.hasConverged())
        return false;
    // N3: score the transform that was actually returned, with the correspondence radius in
    // the same units as setMaxCorrespondenceDistance, and require a whole-scan match - PCL's
    // getFitnessScore(max_range) compares max_range against SQUARED distances and averages
    // over the in-range subset only, so a partial match scores like a full one.
    if (!fitnessAndInliers(m_rough_tree, aligned_cloud, m_config.rough_max_corr_dist,
                           &score, &inlier_ratio))
        return false;
    if (score > m_config.rough_score_thresh || inlier_ratio < m_config.rough_min_inlier_ratio)
        return false;

    m_refine_icp.setInputSource(m_refine_inp);
    m_refine_icp.align(*aligned_cloud, m_rough_icp.getFinalTransformation());
    if (!m_refine_icp.hasConverged())
        return false;
    if (!fitnessAndInliers(m_refine_tree, aligned_cloud, m_config.refine_max_corr_dist,
                           &score, &inlier_ratio))
        return false;
    if (score > m_config.refine_score_thresh || inlier_ratio < m_config.refine_min_inlier_ratio)
        return false;

    guess = m_refine_icp.getFinalTransformation();
    return true;
}