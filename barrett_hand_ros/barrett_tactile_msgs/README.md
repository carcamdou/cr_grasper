# barrett_tactile_msgs
 ROS messages for locations of active tactile sensors 

## ROS Message Format
```C++
Header header
geometry_msgs/PoseStamped[] tactile_info
```
The data comes out as an array of 24 points for each finger and the palm, making it 96 points of information on every message. 