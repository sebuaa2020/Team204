#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include "wpr_object_detect/draw_utils.h"

DrawUtils::DrawUtils() {

}

DrawUtils::DrawUtils(ros::Publisher marker_pub) : marker_pub(marker_pub) {

}

void DrawUtils::RemoveBoxes() {
    line_box.action = 3;
    line_box.points.clear();
    marker_pub.publish(line_box);
}

void DrawUtils::DrawBox(stBoxMarker boxMarker) {
    DrawBoxIntern(boxMarker.xMin, boxMarker.xMax, boxMarker.yMin, boxMarker.yMax, boxMarker.zMin, boxMarker.zMax, 0,
                  1, 0);
}

void
DrawUtils::DrawBoxIntern(float inMinX, float inMaxX, float inMinY, float inMaxY, float inMinZ, float inMaxZ, float inR,
                         float inG, float inB) {
    line_box.header.frame_id = "base_footprint";
    line_box.ns = "line_box";
    line_box.action = visualization_msgs::Marker::ADD;
    line_box.id = 0;
    line_box.type = visualization_msgs::Marker::LINE_LIST;
    line_box.scale.x = 0.005;
    line_box.color.r = inR;
    line_box.color.g = inG;
    line_box.color.b = inB;
    line_box.color.a = 1.0;

    geometry_msgs::Point p;
    p.z = inMinZ;
    p.x = inMinX;
    p.y = inMinY;
    line_box.points.push_back(p);
    p.x = inMinX;
    p.y = inMaxY;
    line_box.points.push_back(p);

    p.x = inMinX;
    p.y = inMaxY;
    line_box.points.push_back(p);
    p.x = inMaxX;
    p.y = inMaxY;
    line_box.points.push_back(p);

    p.x = inMaxX;
    p.y = inMaxY;
    line_box.points.push_back(p);
    p.x = inMaxX;
    p.y = inMinY;
    line_box.points.push_back(p);

    p.x = inMaxX;
    p.y = inMinY;
    line_box.points.push_back(p);
    p.x = inMinX;
    p.y = inMinY;
    line_box.points.push_back(p);

    p.z = inMaxZ;
    p.x = inMinX;
    p.y = inMinY;
    line_box.points.push_back(p);
    p.x = inMinX;
    p.y = inMaxY;
    line_box.points.push_back(p);

    p.x = inMinX;
    p.y = inMaxY;
    line_box.points.push_back(p);
    p.x = inMaxX;
    p.y = inMaxY;
    line_box.points.push_back(p);

    p.x = inMaxX;
    p.y = inMaxY;
    line_box.points.push_back(p);
    p.x = inMaxX;
    p.y = inMinY;
    line_box.points.push_back(p);

    p.x = inMaxX;
    p.y = inMinY;
    line_box.points.push_back(p);
    p.x = inMinX;
    p.y = inMinY;
    line_box.points.push_back(p);

    p.x = inMinX;
    p.y = inMinY;
    p.z = inMinZ;
    line_box.points.push_back(p);
    p.x = inMinX;
    p.y = inMinY;
    p.z = inMaxZ;
    line_box.points.push_back(p);

    p.x = inMinX;
    p.y = inMaxY;
    p.z = inMinZ;
    line_box.points.push_back(p);
    p.x = inMinX;
    p.y = inMaxY;
    p.z = inMaxZ;
    line_box.points.push_back(p);

    p.x = inMaxX;
    p.y = inMaxY;
    p.z = inMinZ;
    line_box.points.push_back(p);
    p.x = inMaxX;
    p.y = inMaxY;
    p.z = inMaxZ;
    line_box.points.push_back(p);

    p.x = inMaxX;
    p.y = inMinY;
    p.z = inMinZ;
    line_box.points.push_back(p);
    p.x = inMaxX;
    p.y = inMinY;
    p.z = inMaxZ;
    line_box.points.push_back(p);
    marker_pub.publish(line_box);
}