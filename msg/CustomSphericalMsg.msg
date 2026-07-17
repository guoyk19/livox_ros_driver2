# Livox spherical point cloud frame.

std_msgs/Header header              # ROS standard message header
uint64 timebase                     # Timestamp of the first point, unit:ns
uint32 point_num                    # Total number of points
uint8 lidar_id                      # LiDAR device id
uint8[3] rsvd                       # Reserved
CustomSphericalPoint[] points       # Spherical point cloud data
