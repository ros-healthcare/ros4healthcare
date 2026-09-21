# PupilGaze.msg
std_msgs/Header header
float64 gaze_x
float64 gaze_y
bool worn

# Pupil Diameters
float64 pupil_diameter_left
float64 pupil_diameter_right

# 3D Eyeball Center (Left)
float64 eyeball_center_left_x
float64 eyeball_center_left_y
float64 eyeball_center_left_z

# 3D Optical Axis (Left)
float64 optical_axis_left_x
float64 optical_axis_left_y
float64 optical_axis_left_z

# 3D Eyeball Center (Right)
float64 eyeball_center_right_x
float64 eyeball_center_right_y
float64 eyeball_center_right_z

# 3D Optical Axis (Right)
float64 optical_axis_right_x
float64 optical_axis_right_y
float64 optical_axis_right_z

# Eyelid State
float64 eyelid_angle_top_left
float64 eyelid_angle_bottom_left
float64 eyelid_aperture_left

float64 eyelid_angle_top_right
float64 eyelid_angle_bottom_right
float64 eyelid_aperture_right
