## Topic

### Subscribe

| Topic              | Type                     | Info                          |
| ------------------ | ------------------------ | ----------------------------- |
| /kinect2/sd/points | sensor_msgs::PointCloud2 | Pointcloud from kinect camera |

### Publish

| Topic         | Type                        | Info                        |
| ------------- | --------------------------- | --------------------------- |
| /obj\_marker  | visualization_msgs::Marker  | Marker for Rviz             |
| /box\_plane   | std_msgs::Float32MultiArray | Bounding Box of plane       |
| /box\_objects | std_msgs::Float32MultiArray | Bounding Box of all objects |

## Note

Mapping from `std_msgs::Float32MultiArray` to Bounding Box:

| Index | Field |
| ----- | ----- |
| 0     | xMin  |
| 1     | xMax  |
| 2     | yMin  |
| 3     | yMax  |
| 4     | zMin  |
| 5     | zMax  |

Size of the array must be a multiple of 6.