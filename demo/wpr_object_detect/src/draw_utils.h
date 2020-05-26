#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

/** \brief Bounding box
 */
struct stBoxMarker {
    float xMax;
    float xMin;
    float yMax;
    float yMin;
    float zMax;
    float zMin;
};

/** \brief DrawUtils
 */
class DrawUtils {
private:
    visualization_msgs::Marker line_box;
    ros::Publisher marker_pub;

    void DrawBoxIntern(float inMinX, float inMaxX, float inMinY, float inMaxY, float inMinZ, float inMaxZ, float inR,
                       float inG, float inB);

public:
    /** \brief constructor
     * \param marker_pub Marker publisher
     */
    DrawUtils(ros::Publisher marker_pub);
    /** \brief constructor
     */
    DrawUtils();

    /** \brief Draw the bounding box
     * \param boxMarker box
     */
    void DrawBox(stBoxMarker boxMarker);

    /** \brief Remove all boxes
     */
    void RemoveBoxes();
};