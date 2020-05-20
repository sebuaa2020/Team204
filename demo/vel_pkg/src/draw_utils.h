#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
struct stBoxMarker {
    float xMax;
    float xMin;
    float yMax;
    float yMin;
    float zMax;
    float zMin;
};

class DrawUtils {
private:
    visualization_msgs::Marker line_box;
    ros::Publisher marker_pub;

    void DrawBoxIntern(float inMinX, float inMaxX, float inMinY, float inMaxY, float inMinZ, float inMaxZ, float inR,
                       float inG, float inB);

public:
    DrawUtils(ros::Publisher marker_pub);
    DrawUtils();
    void DrawBox(stBoxMarker boxMarker);

    void RemoveBoxes();
};