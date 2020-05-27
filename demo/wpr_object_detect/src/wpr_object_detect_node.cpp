//
// Created by yspjack on 2020/5/26.
//

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include "wpr_object_detect/draw_utils.h"
#include "wpr_object_detect/object_detect.h"
#include <std_msgs/Float32MultiArray.h>

static int nCount = 0;
static std::shared_ptr<tf::TransformListener> tf_listener;
static std::shared_ptr<DrawUtils> drawUtils;
static stBoxMarker boxMarker;

static ros::Publisher pub_planar;
static ros::Publisher pub_objects;
static ros::Publisher marker_pub;
static ObjectDetect objectDetect;

void calcBoxMarker(pcl::PointCloud<PointT>::Ptr cloud, std_msgs::Float32MultiArray &msg) {
    bool bFirstPoint = true;
    for (const auto &p: cloud->points) {
        if (bFirstPoint == true) {
            boxMarker.xMax = boxMarker.xMin = p.x;
            boxMarker.yMax = boxMarker.yMin = p.y;
            boxMarker.zMax = boxMarker.zMin = p.z;
            bFirstPoint = false;
        }
        if (p.x < boxMarker.xMin) { boxMarker.xMin = p.x; }
        if (p.x > boxMarker.xMax) { boxMarker.xMax = p.x; }
        if (p.y < boxMarker.yMin) { boxMarker.yMin = p.y; }
        if (p.y > boxMarker.yMax) { boxMarker.yMax = p.y; }
        if (p.z < boxMarker.zMin) { boxMarker.zMin = p.z; }
        if (p.z > boxMarker.zMax) { boxMarker.zMax = p.z; }
    }
    msg.data.push_back(boxMarker.xMin);
    msg.data.push_back(boxMarker.xMax);
    msg.data.push_back(boxMarker.yMin);
    msg.data.push_back(boxMarker.yMax);
    msg.data.push_back(boxMarker.zMin);
    msg.data.push_back(boxMarker.zMax);
}

void ProcCloudCB(const sensor_msgs::PointCloud2 &input) {
    ROS_INFO("ProcCloudCB: %d, %d", input.height, input.width);
    bool r;
    r = tf_listener->waitForTransform("/base_footprint", input.header.frame_id, input.header.stamp, ros::Duration(3.0));
    if (!r) {
        ROS_INFO("waitForTransform");
        return;
    }
    sensor_msgs::PointCloud2 pc_footprint;
    pcl_ros::transformPointCloud("/base_footprint", input, pc_footprint, *tf_listener);

    pcl::PointCloud<PointT> cloud_src;
    pcl::fromROSMsg(pc_footprint, cloud_src);
    pcl::PointCloud<PointT>::Ptr cloud_source_ptr;
    cloud_source_ptr = cloud_src.makeShared();

    objectDetect.setInputCloud(cloud_source_ptr);

    pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
    r = objectDetect.detectPlane(cloud_plane);
    if (!r) {
        ROS_WARN("Plane not detected");
        return;
    }

    std_msgs::Float32MultiArray boxMsg;
    boxMsg.data.clear();
    calcBoxMarker(cloud_plane, boxMsg);
    pub_planar.publish(boxMsg);

    std::vector<pcl::PointCloud<PointT>> clusters_cloud;
    r = objectDetect.detectObject(clusters_cloud);
    drawUtils->RemoveBoxes();

    drawUtils->DrawBox(boxMarker);
    if (!r) {
        ROS_WARN("Object not detected");
        return;
    }
    boxMsg.data.clear();
    int j = 0;
    for (const auto &cluster : clusters_cloud) {
        j++;
        ROS_INFO_STREAM("PointCloud representing the Cluster: " << cluster.points.size() << " data points.");
        calcBoxMarker(boost::make_shared<pcl::PointCloud<PointT>>(cluster), boxMsg);
        drawUtils->DrawBox(boxMarker);
    }
    pub_objects.publish(boxMsg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "wpr_object_detect_node");
    ros::NodeHandle nh;
    ros::Subscriber pc_sub = nh.subscribe("/cloud_pcd", 1, ProcCloudCB);
    marker_pub = nh.advertise<visualization_msgs::Marker>("obj_marker", 10);

    pub_planar = nh.advertise<std_msgs::Float32MultiArray>("box_plane", 1);
    pub_objects = nh.advertise<std_msgs::Float32MultiArray>("box_objects", 1);

    ROS_INFO("wpr_object_detect_node");
    tf_listener = std::make_shared<tf::TransformListener>();
    drawUtils = std::make_shared<DrawUtils>(marker_pub);
    ros::spin();
    return 0;
}