// ===========================================================================
// VENDORED THIRD-PARTY FILE -- Scan Context global LiDAR descriptor.
//
// Upstream : https://github.com/gisbi-kim/SC-LIO-SAM  (branch master)
// Commit   : d43ca00d97a756303c10975e32e8d66bfabb337d
// Files    : SC-LIO-SAM/include/Scancontext.h , SC-LIO-SAM/src/Scancontext.cpp
//            (SC-LIO-SAM vendors the irapkaist/scancontext descriptor pair
//             unchanged; the original repository https://github.com/irapkaist/
//             scancontext returns HTTP 404 as of 2026-09-28, verified with
//             `git ls-remote`.  Backup mirror: gisbi-kim/scancontext @ 93672835.)
// Paper    : G. Kim and A. Kim, "Scan Context: Egocentric Spatial Descriptor for
//            Place Recognition within 3D Point Cloud Map", IROS 2018.
// License  : CC BY-NC-SA 4.0 (Creative Commons Attribution-NonCommercial-
//            ShareAlike 4.0 International), per the upstream README
//            "## License / ### Copyright": "All codes on this page are
//            copyrighted by KAIST and Naver Labs ... You may not use the work
//            for commercial purposes, and you may only distribute the resulting
//            work under the same license if you alter, transform, or create the
//            work."  NON-COMMERCIAL USE ONLY -- see
//            /home/lee/t21_wp2/reports/step12_scancontext_gicp.md section
//            "Provenance and licensing" for the consequences for this fork.
// sha256   : 5a7204f5e8d14c0105dbb699bf5182d5c7af6de04e1a426bcc26a71df6248484
// Local modifications, all confined to Scancontext.h:
//   1. the four descriptor hyper-parameters (LIDAR_HEIGHT, PC_NUM_RING,
//      PC_NUM_SECTOR, PC_MAX_RADIUS) were changed from `const` to mutable and a
//      parameterised constructor was added, so the descriptor geometry can be
//      configured from pgo.yaml.  The default member initialisers are the
//      upstream values (20 rings x 60 sectors, 80 m, 2 m), so the byte-for-byte
//      behaviour with the default constructor is unchanged.
//   2. nothing else in this file or in Scancontext.cpp was touched.
// ===========================================================================
#pragma once

#include <ctime>
#include <cassert>
#include <cmath>
#include <utility>
#include <vector>
#include <algorithm> 
#include <cstdlib>
#include <memory>
#include <iostream>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include "nanoflann.hpp"
#include "KDTreeVectorOfVectorsAdaptor.h"

#include "tictoc.h"

using namespace Eigen;
using namespace nanoflann;

using std::cout;
using std::endl;
using std::make_pair;

using std::atan2;
using std::cos;
using std::sin;

using SCPointType = pcl::PointXYZI; // using xyz only. but a user can exchange the original bin encoding function (i.e., max hegiht) to max intensity (for detail, refer 20 ICRA Intensity Scan Context)
using KeyMat = std::vector<std::vector<float> >;
using InvKeyTree = KDTreeVectorOfVectorsAdaptor< KeyMat, float >;


// namespace SC2
// {

void coreImportTest ( void );


// sc param-independent helper functions 
float xy2theta( const float & _x, const float & _y );
MatrixXd circshift( MatrixXd &_mat, int _num_shift );
std::vector<float> eig2stdvec( MatrixXd _eigmat );


class SCManager
{
public: 
    SCManager( ) = default; // reserving data space (of std::vector) could be considered. but the descriptor is lightweight so don't care.
    // FORK ADDITION (see the provenance block at the top of this file): the four descriptor
    // hyper-parameters below are settable from the outside so the Scan Context geometry can be
    // driven by pgo.yaml.  SCManager() above reproduces the upstream defaults exactly.
    SCManager( double _lidar_height, int _num_ring, int _num_sector, double _max_radius )
        : LIDAR_HEIGHT( _lidar_height )
        , PC_NUM_RING( _num_ring )
        , PC_NUM_SECTOR( _num_sector )
        , PC_MAX_RADIUS( _max_radius )
    {} // member initialisation order == declaration order, so PC_UNIT_* derive from the new values

    Eigen::MatrixXd makeScancontext( pcl::PointCloud<SCPointType> & _scan_down );
    Eigen::MatrixXd makeRingkeyFromScancontext( Eigen::MatrixXd &_desc );
    Eigen::MatrixXd makeSectorkeyFromScancontext( Eigen::MatrixXd &_desc );

    int fastAlignUsingVkey ( MatrixXd & _vkey1, MatrixXd & _vkey2 ); 
    double distDirectSC ( MatrixXd &_sc1, MatrixXd &_sc2 ); // "d" (eq 5) in the original paper (IROS 18)
    std::pair<double, int> distanceBtnScanContext ( MatrixXd &_sc1, MatrixXd &_sc2 ); // "D" (eq 6) in the original paper (IROS 18)

    // User-side API
    void makeAndSaveScancontextAndKeys( pcl::PointCloud<SCPointType> & _scan_down );
    std::pair<int, float> detectLoopClosureID( void ); // int: nearest node index, float: relative yaw  

    // for ltmapper 
    const Eigen::MatrixXd& getConstRefRecentSCD(void);

public:
    // hyper parameters ()
    double LIDAR_HEIGHT = 2.0; // lidar height : add this for simply directly using lidar scan in the lidar local coord (not robot base coord) / if you use robot-coord-transformed lidar scans, just set this as 0.

    int    PC_NUM_RING = 20; // 20 in the original paper (IROS 18)
    int    PC_NUM_SECTOR = 60; // 60 in the original paper (IROS 18)
    double PC_MAX_RADIUS = 80.0; // 80 meter max in the original paper (IROS 18)
    double PC_UNIT_SECTORANGLE = 360.0 / double(PC_NUM_SECTOR);
    double PC_UNIT_RINGGAP = PC_MAX_RADIUS / double(PC_NUM_RING);

    // tree
    int    NUM_EXCLUDE_RECENT = 30; // simply just keyframe gap (related with loopClosureFrequency in yaml), but node position distance-based exclusion is ok. 
    int    NUM_CANDIDATES_FROM_TREE = 3; // 10 is enough. (refer the IROS 18 paper)

    // loop thres
    double SEARCH_RATIO = 0.1; // for fast comparison, no Brute-force, but search 10 % is okay. // not was in the original conf paper, but improved ver.
    // const double SC_DIST_THRES = 0.13; // empirically 0.1-0.2 is fine (rare false-alarms) for 20x60 polar context (but for 0.15 <, DCS or ICP fit score check (e.g., in LeGO-LOAM) should be required for robustness)
    double SC_DIST_THRES = 0.3; // 0.4-0.6 is good choice for using with robust kernel (e.g., Cauchy, DCS) + icp fitness threshold / if not, recommend 0.1-0.15
    // const double SC_DIST_THRES = 0.7; // 0.4-0.6 is good choice for using with robust kernel (e.g., Cauchy, DCS) + icp fitness threshold / if not, recommend 0.1-0.15

    // config 
    int    TREE_MAKING_PERIOD_ = 10; // i.e., remaking tree frequency, to avoid non-mandatory every remaking, to save time cost / in the LeGO-LOAM integration, it is synchronized with the loop detection callback (which is 1Hz) so it means the tree is updated evrey 10 sec. But you can use the smaller value because it is enough fast ~ 5-50ms wrt N.
    int          tree_making_period_conter = 0;

    // data 
    std::vector<double> polarcontexts_timestamp_; // optional.
    std::vector<Eigen::MatrixXd> polarcontexts_;
    std::vector<Eigen::MatrixXd> polarcontext_invkeys_;
    std::vector<Eigen::MatrixXd> polarcontext_vkeys_;

    KeyMat polarcontext_invkeys_mat_;
    KeyMat polarcontext_invkeys_to_search_;
    std::unique_ptr<InvKeyTree> polarcontext_tree_;

}; // SCManager

// } // namespace SC2
