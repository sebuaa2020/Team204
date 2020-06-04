//
// Created by yspjack on 2020/5/19.
//

#ifndef SRC_OBJECT_DETECT_H
#define SRC_OBJECT_DETECT_H

// ObjectDetect

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <map>

typedef pcl::PointXYZ PointT;

/** \brief Object detection module
 */
class ObjectDetect {
private:
    std::pair<float, float> xPassThroughLimit;
    std::pair<float, float> yPassThroughLimit;
    float zPassThroughLimitMax;
    float zPassThroughLimitMin;
    pcl::PointCloud<PointT>::Ptr cloud_filtered;
    pcl::PointCloud<PointT>::Ptr cloud_hull;
    bool isDetectedPlane;

public:
    /** \brief constructor
     */
    ObjectDetect();

    /** \brief set PassThrough filter limits
     * \param limit_min The minimum allowed x value
     * \param limit_max The maximum allowed x value
     */
    void setxLimits(const float &limit_min, const float &limit_max);

    /** \brief set PassThrough filter limits
     * \param limit_min The minimum allowed y value
     * \param limit_max The maximum allowed y value
     */
    void setyLimits(const float &limit_min, const float &limit_max);

    /** \brief set PassThrough filter limits
     * \param limit_min The minimum allowed z value
     * \param limit_max The maximum allowed z value
     */
    void setzLimits(const float &limit_min, const float &limit_max);

    /**
     * \brief set input point cloud
     * @param cloud input point cloud
     */
    void setInputCloud(pcl::PointCloud<PointT>::Ptr cloud);

    /** \brief detect plane from given point cloud
     * \param cloud_plane output point cloud of plane
     * \retval true success
     * \retval false otherwise
     */
    bool detectPlane(pcl::PointCloud<PointT>::Ptr cloud_plane);

    /** \brief detect objects from given point cloud
     * \param clusters_cloud array of object point clouds
     * \retval true success
     * \retval false otherwise
     */
    bool detectObject(std::vector<pcl::PointCloud<PointT>> &clusters_cloud);

    /** \brief get filtered point cloud
     * \return point cloud
     */
    pcl::PointCloud<PointT>::Ptr getFilteredCloud();
};


#endif //SRC_OBJECT_DETECT_H
