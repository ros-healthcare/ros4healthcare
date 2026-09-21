# Fixation.msg
std_msgs/Header header
float64 duration              # Calculated: (end_time_ns - start_time_ns) / 1e9
int64 start_time_ns
int64 end_time_ns
float64 start_gaze_x
float64 start_gaze_y
float64 end_gaze_x
float64 end_gaze_y
float64 mean_gaze_x
float64 mean_gaze_y
float64 amplitude_pixels
float64 amplitude_angle_deg
float64 mean_velocity
float64 max_velocity
