#pragma once

#define _CRT_SECURE_NO_WARNINGS

#include <vector>
#include <math.h>
#include <limits.h>
#include <queue>
#include <algorithm>
#include <iostream>
#include <string>
#include <cstdlib>
#include <fstream>
#include <set>
#include <cassert>
#include <algorithm>
#include <boost/filesystem.hpp>
#include <ctime>

#include <iomanip>

#include <unsupported/Eigen/MatrixFunctions>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <direct.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/features/fpfh.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_iterator.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/octree/octree_search.h>

#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/don.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/local_maximum.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply/ply.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/point_types.h>
#include <pcl/surface/poisson.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <boost/function_output_iterator.hpp>

#define OCTOMAP_DISABLE_WARNING 
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include <octomap/octomap_types.h>
#include <octomap/octomap.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>

#include <embree3/rtcore.h>


//laslib related
//#include <lasreader.hpp>

// CGAL includes.
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_set_3.h>
#include <CGAL\Shape_detection_3.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/property_map.h>
#include <CGAL/IO/read_xyz_points.h>


#include <CGAL/Alpha_shape_3.h>
#include <CGAL/Alpha_shape_cell_base_3.h>
#include <CGAL/Alpha_shape_vertex_base_3.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <cassert>

#include <CGAL/algorithm.h>
#include <CGAL/assertions.h>


namespace fs = boost::filesystem;

//for CGAl
typedef CGAL::Exact_predicates_inexact_constructions_kernel kernel;
typedef kernel::FT FT;
typedef kernel::Point_3 Pointn;
typedef kernel::Vector_3 Vector;
typedef CGAL::Point_set_3<Pointn> Point_set;

typedef std::pair<kernel::Point_3, kernel::Vector_3>         Point_with_normal;
typedef std::vector<Point_with_normal>                       Pwn_vector;
typedef CGAL::First_of_pair_property_map<Point_with_normal>  Point_map;
typedef CGAL::Second_of_pair_property_map<Point_with_normal> Normal_map;

typedef CGAL::Shape_detection_3::Shape_detection_traits
<kernel, Pwn_vector, Point_map, Normal_map>                Traits;
typedef CGAL::Shape_detection_3::Efficient_RANSAC<Traits>    Efficient_ransac;
typedef CGAL::Shape_detection_3::Region_growing_depr<Traits>      Region_growing;
typedef CGAL::Shape_detection_3::Plane<Traits>               Plane;

//Type definition
typedef  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PCXYZRGBAPtr;
typedef  pcl::PointCloud<pcl::PointXYZRGBA> PCXYZRGBA;
typedef  pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCXYZRGBPtr;
typedef  pcl::PointCloud<pcl::PointXYZRGB> PCXYZRGB;
typedef  pcl::PointCloud<pcl::PointXYZ>::Ptr PCXYZPtr;
typedef  pcl::PointCloud<pcl::PointXYZ> PCXYZ;
typedef  pcl::PointCloud<pcl::PointXYZL>::Ptr PCXYZLPtr;
typedef  pcl::PointCloud<pcl::PointXYZL> PCXYZL;
typedef  pcl::PointCloud<pcl::PointXYZI>::Ptr PCXYZIPtr;
typedef  pcl::PointCloud<pcl::PointXYZI> PCXYZI;

typedef  pcl::PointCloud<pcl::Normal>::Ptr PTNORMPtr;
typedef  pcl::PointCloud<pcl::Normal> PTNORM;


struct myPoint {
    int pointID;
    int imageID;
    float intensity;
    float distance;
    float angle;

    // ∂®“Â≈≈–Ú∫Ø ˝
    bool operator<(const myPoint& other) const {
        if (pointID < other.pointID) {
            return true;
        }
        else if (pointID == other.pointID) {
            return imageID < other.imageID;
        }
        return false;
    }
};

struct Pos {
    int frame;
    cv::Point3f T;
    cv::Point3f R;
};

bool compareAngle(const myPoint& a, const myPoint& b) {
    if (a.pointID < b.pointID) {
        return true;
    }
    else if (a.pointID > b.pointID) {
        return false;
    }
    else {
        return a.angle < b.angle;
    }
}
//    std::sort(data.begin(), data.end(), compareData);

bool compareDistance(const myPoint& a, const myPoint& b) {
    if (a.pointID < b.pointID) {
        return true;
    }
    else if (a.pointID > b.pointID) {
        return false;
    }
    else {
        return a.angle < b.angle;
    }
}//    std::sort(data.begin(), data.end(), compareData);

double interpolate(double x, const std::vector<double>& xs, const std::vector<double>& ys)
{
    // Find the lower bound index
    auto it = lower_bound(xs.begin(), xs.end(), x);
    int i = it - xs.begin();

    // Check if x is outside of the range of xs
    if (i == 0) {
        return ys[0];
    }
    else if (i == xs.size()) {
        return ys.back();
    }

    // Interpolate
    double x0 = xs[i - 1];
    double x1 = xs[i];
    double y0 = ys[i - 1];
    double y1 = ys[i];
    double y = y0 + (y1 - y0) * (x - x0) / (x1 - x0);
    return y;
}

struct Vprop {
    double intensity;
    double dist;
    double angle;
    int label = 0;
    int imgid = 0;
};

/*	bool compareByDist(const Voxelprop& a, const Voxelprop& b)
{
    return a.dist < b.dist;
}

bool compareByAngle(const Voxelprop& a, const Voxelprop& b)
{
    return a.angle < b.angle;
}*/

bool compareByD(const Vprop& a, const Vprop& b)
{
    return a.dist < b.dist;
}

bool compareByA(const Vprop& a, const Vprop& b)
{
    return a.angle < b.angle;
}

//struct CompareByDist {
//    bool operator()(const Vprop& a, const Vprop& b) const {
//        return a.dist < b.dist;
//    }
//};
//
//struct VoxelPropsD {
//    pcl::PointXYZRGBL pt;
//    //vector<Vprop> props;
//    std::set<Vprop, CompareByDist> propsD;
//};
//
//struct CompareByAngle {
//    bool operator()(const Vprop& a, const Vprop& b) const {
//        return a.angle < b.angle;
//    }
//};

//struct VoxelPropsA {
//    pcl::PointXYZRGBL pt;
//    //vector<Vprop> props;
//    std::set<Vprop, CompareByAngle> propsA;
//};
//
struct VoxelProps {
    pcl::PointXYZRGBL pt;
    //vector<Vprop> props;
    std::vector<Vprop> props;
};
// Custom comparison function for OcTreeKey
struct KeyCompare {
    bool operator()(const octomap::OcTreeKey& lhs, const octomap::OcTreeKey& rhs) const {
        // Compare the keys component-wise
        if (lhs[0] < rhs[0]) return true;
        if (lhs[0] > rhs[0]) return false;
        if (lhs[1] < rhs[1]) return true;
        if (lhs[1] > rhs[1]) return false;
        return lhs[2] < rhs[2];
    }
};



//vector<double> xs = { 1.0, 2.0, 3.0, 4.0, 5.0 };
//vector<double> ys = { 2.0, 3.0, 1.0, 4.0, 5.0 };
//
//double x = 2.5;
//double y = interpolate(x, xs, ys);
