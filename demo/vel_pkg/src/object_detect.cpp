// ObjectDetect
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

typedef pcl::PointXYZ PointT;

/** \brief Object detection module
    */
class ObjectDetect {
private:
    float zPassThroughLimitMax;
    float zPassThroughLimitMin;
    pcl::PointCloud<PointT>::Ptr cloud_filtered;
    bool isDetectedPlane;

public:
    /** \brief constructor
     */
    ObjectDetect() : zPassThroughLimitMin(0.0), zPassThroughLimitMax(1.5) {
        cloud_filtered = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    }

    /** \brief set PassThrough filter limits
     * \param limit_min The minimum allowed z value
     * \param limit_max The maximum allowed z value
     */
    void setzLimits(const float &limit_min, const float &limit_max) {
        zPassThroughLimitMin = limit_min;
        zPassThroughLimitMax = limit_max;
    }

    /**
     * \brief set input point cloud
     * @param cloud input point cloud
     */
    void setInputCloud(pcl::PointCloud<PointT>::Ptr cloud) {
        isDetectedPlane = false;
        // Create the filtering object: downsample the dataset using a leaf size of 0.5cm
        pcl::VoxelGrid<PointT> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.005f, 0.005f, 0.005f);
        vg.filter(*cloud_filtered);
        std::cout << "PointCloud after VoxelGrid has: " << cloud_filtered->points.size() << " data points."
                  << std::endl;

        pcl::PassThrough<PointT> pass;
        pass.setInputCloud(cloud_filtered);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(zPassThroughLimitMin, zPassThroughLimitMax);
        pass.filter(*cloud_filtered);
        std::cout << "PointCloud after PassThrough has: " << cloud_filtered->points.size() << " data points."
                  << std::endl;
    }

    /** \brief detect plane from given point cloud
     * \param cloud_plane output point cloud of plane
     * \retval true success
     * \retval false otherwise
     */
    bool detectPlane(pcl::PointCloud<PointT>::Ptr cloud_plane) {
        pcl::PointCloud<PointT>::Ptr cloud_f(new pcl::PointCloud<PointT>);
        // Create the segmentation object for the planar model and set all the parameters
        pcl::SACSegmentation<PointT> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(0.02);

        int i = 0, nr_points = (int) cloud_filtered->points.size();
        while (cloud_filtered->points.size() > 0.3 * nr_points) {
            // Segment the largest planar component from the remaining cloud
            seg.setInputCloud(cloud_filtered);
            seg.segment(*inliers, *coefficients);
            if (inliers->indices.size() == 0) {
                std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
                // break;
                return false;
            }

            // Extract the planar inliers from the input cloud
            pcl::ExtractIndices<pcl::PointXYZ> extract;
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
        isDetectedPlane = true;
        return true;
    }

    /** \brief detect objects from given point cloud
     * \param clusters_cloud array of object point clouds
     * \retval true success
     * \retval false otherwise
     */
    bool detectObject(std::vector<pcl::PointCloud<PointT>> &clusters_cloud) {
        if (!isDetectedPlane) {
            std::cout << "Plane not detected" << std::endl;
            return false;
        }
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        tree->setInputCloud(cloud_filtered);
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(0.02);
        ec.setMinClusterSize(1000);
        ec.setMaxClusterSize(50000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_filtered);

        std::vector<pcl::PointIndices> cluster_indices;
        ec.extract(cluster_indices);
        if (cluster_indices.size() == 0) {
            return false;
        }

        int j = 0;
        pcl::ExtractIndices<PointT> extract;
        for (const auto it : cluster_indices) {
            pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
            extract.setInputCloud(cloud_filtered);
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
};
