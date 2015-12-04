# A view point in an entropy map
# author: Tim Patten, t.patten@acfr.usyd.edu.au

sensor_msgs/PointCloud2 cloud
std_msgs/String class_type
std_msgs/String instance_name
std_msgs/String view_name
std_msgs/Float64MultiArray transform
float32[] centroid
geometry_msgs/Pose camera_pose
float64 surface_area_proportion
float64 class_entropy
float64 recognition_probability
