#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>

static std::string pub_topic;
class CWPHLidarFilter
{
public:
    CWPHLidarFilter();
private:
     ros::NodeHandle n;
     ros::Publisher scan_pub;
     ros::Subscriber scan_sub;
     void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
};

CWPHLidarFilter::CWPHLidarFilter()
{
    scan_pub = n.advertise<sensor_msgs::LaserScan>(pub_topic,1);
    scan_sub = n.subscribe<sensor_msgs::LaserScan>("/scan_raw",1,&CWPHLidarFilter::lidarCallback,this);
}

void CWPHLidarFilter::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    //ROS_INFO("[wpb_home_lidar_filter]");
    int nRanges = scan->ranges.size();
    sensor_msgs::LaserScan new_scan;
    new_scan.header.stamp = scan->header.stamp;
    new_scan.header.frame_id = scan->header.frame_id;
    new_scan.angle_max = scan->angle_max;
    new_scan.angle_min = scan->angle_min;
    new_scan.angle_increment = scan->angle_increment;
    new_scan.time_increment = scan->time_increment;
    new_scan.range_min = 0.25;
    new_scan.range_max = scan->range_max;
    new_scan.ranges.resize(nRanges);
    new_scan.intensities.resize(nRanges);
    for(int i=0 ; i<nRanges ; i++)
    {
        new_scan.ranges[i] = scan->ranges[i];
        if(new_scan.ranges[i] < 0.25)
        {
            new_scan.ranges[i] = new_scan.range_max+1.0;
        }
        new_scan.intensities[i] = scan->intensities[i];
    }
    scan_pub.publish(new_scan);
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"lidar_filter");

     ros::NodeHandle n_param("~");
    std::string strSerialPort;
    n_param.param<std::string>("pub_topic", pub_topic, "/scan_filtered");

    CWPHLidarFilter lidar_filter;
    ros::spin();
}