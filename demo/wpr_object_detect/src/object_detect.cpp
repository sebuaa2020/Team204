// ObjectDetect
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include "wpr_object_detect/object_detect.h"


ObjectDetect::ObjectDetect() : zPassThroughLimitMin(0.1), zPassThroughLimitMax(1.5) {
    cloud_filtered = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    cloud_hull = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    isDetectedPlane = false;
}


void ObjectDetect::setzLimits(const float &limit_min, const float &limit_max) {
    zPassThroughLimitMin = limit_min;
    zPassThroughLimitMax = limit_max;
}

void ObjectDetect::setInputCloud(pcl::PointCloud<PointT>::Ptr cloud) {
    isDetectedPlane = false;
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.0, 1.5);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-1.5, 1.5);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(zPassThroughLimitMin, zPassThroughLimitMax);
    pass.filter(*cloud_filtered);
    std::cout << "PointCloud after PassThrough has: " << cloud_filtered->points.size() << " data points."
              << std::endl;
    // Create the filtering object: downsample the dataset using a leaf size of 0.5cm
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud_filtered);
    vg.setLeafSize(0.005f, 0.005f, 0.005f);
    vg.filter(*cloud_filtered);
    std::cout << "PointCloud after VoxelGrid has: " << cloud_filtered->points.size() << " data points."
              << std::endl;
}

bool ObjectDetect::detectPlane(pcl::PointCloud<PointT>::Ptr cloud_plane) {
    if (cloud_filtered->points.empty()) {
        return false;
    }
    pcl::PointCloud<PointT>::Ptr cloud_f(new pcl::PointCloud<PointT>);
    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.03);
    Eigen::Vector3f axis = Eigen::Vector3f(0.0, 0.0, 1.0);
    seg.setAxis(axis);
    seg.setEpsAngle(15.0f * (M_PI / 180.0f));

    int i = 0, nr_points = (int) cloud_filtered->points.size();
    while (cloud_filtered->points.size() > 0.5 * nr_points) {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0) {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
//            return false;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);

        // Get the points associated with the planar surface
        extract.filter(*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size()
                  << " data points."
                  << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative(true);
        extract.filter(*cloud_f);
        *cloud_filtered = *cloud_f;
    }
    if (cloud_plane->points.empty()) {
        return false;
    }
    pcl::ConvexHull<PointT> hull;
    hull.setInputCloud(cloud_plane);
    hull.setDimension(2);
    hull.reconstruct(*cloud_hull);
    if (hull.getDimension() != 2) {
        return false;
    }
    if (cloud_hull->points.empty()) {
        return false;
    }
    isDetectedPlane = true;
    return true;
}

bool ObjectDetect::detectObject(std::vector<pcl::PointCloud<PointT>> &clusters_cloud) {
    if (!isDetectedPlane) {
        std::cout << "Plane not detected" << std::endl;
        return false;
    }

    pcl::ExtractPolygonalPrismData<PointT> prism;
    prism.setInputCloud(cloud_filtered);
    prism.setInputPlanarHull(cloud_hull);
    prism.setHeightLimits(-0.50, -0.01); //height limit objects lying on the plane

    pcl::PointIndices::Ptr objectIndices(new pcl::PointIndices);
    pcl::PointCloud<PointT>::Ptr cloud_objects(new pcl::PointCloud<PointT>());
    prism.segment(*objectIndices);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(objectIndices);
    extract.filter(*cloud_objects);

    if (cloud_objects->points.empty()) {
        return false;
    }

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud_objects);
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(0.03);
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_objects);

    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract(cluster_indices);
    if (cluster_indices.empty()) {
        return false;
    }

    int j = 0;
    for (const auto it : cluster_indices) {
        pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        extract.setInputCloud(cloud_objects);
        extract.setNegative(false);
        extract.setIndices(boost::make_shared<const pcl::PointIndices>(it));
        extract.filter(*cloud_cluster);
        clusters_cloud.push_back(*cloud_cluster);

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points."
                  << std::endl;
        j++;
    }
    return true;
}