# Target pose is expressed in the configured base_frame. Position is in metres;
# orientation is a quaternion. Speed/acceleration are in m/s and m/s^2.
string command_id
geometry_msgs/PoseStamped target
float64 speed
float64 acceleration
float64 blend_radius
