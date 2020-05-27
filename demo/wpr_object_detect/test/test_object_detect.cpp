//
// Created by yspjack on 2020/5/27.
//

#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include "wpr_object_detect/object_detect.h"
#include <unistd.h>

ObjectDetect objectDetect;
std::string home_dir;

TEST(TestSuite, testCase1) {
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>);
    std::vector<pcl::PointCloud<PointT>> clusters_cloud;
    pcl::PCDReader reader;
    reader.read(home_dir + "/demo_ws/base_footprint/objects.pcd", *cloud);
    objectDetect.setInputCloud(cloud);
    EXPECT_TRUE(objectDetect.detectPlane(cloud_plane));
    EXPECT_TRUE(objectDetect.detectObject(clusters_cloud));
    EXPECT_EQ(2, clusters_cloud.size());
}

TEST(TestSuite, testCase2) {
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>);
    std::vector<pcl::PointCloud<PointT>> clusters_cloud;
    pcl::PCDReader reader;
    reader.read(home_dir + "/demo_ws/base_footprint/table.pcd", *cloud);
    objectDetect.setInputCloud(cloud);
    EXPECT_TRUE(objectDetect.detectPlane(cloud_plane));
    EXPECT_FALSE(objectDetect.detectObject(clusters_cloud));
    EXPECT_EQ(0, clusters_cloud.size());
}

TEST(TestSuite, testCase3) {
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>);
    std::vector<pcl::PointCloud<PointT>> clusters_cloud;
    pcl::PCDReader reader;
    reader.read(home_dir + "/demo_ws/base_footprint/none.pcd", *cloud);
    objectDetect.setInputCloud(cloud);
    EXPECT_FALSE(objectDetect.detectPlane(cloud_plane));
    EXPECT_FALSE(objectDetect.detectObject(clusters_cloud));
    EXPECT_EQ(0, clusters_cloud.size());
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    home_dir = std::string(getenv("HOME"));

    return RUN_ALL_TESTS();
}
