# Current autopilot estimator state
#
# https://mavlink.io/en/messages/common.html#ESTIMATOR_STATUS_FLAGS

std_msgs/Header header
bool attitude_status_flag

bool velocity_horiz_status_flag
bool velocity_vert_status_flag

bool pos_horiz_rel_status_flag
bool pos_horiz_abs_status_flag

bool pos_vert_abs_status_flag
bool pos_vert_agl_status_flag

bool const_pos_mode_status_flag

bool pred_pos_horiz_rel_status_flag
bool pred_pos_horiz_abs_status_flag

bool gps_glitch_status_flag
bool accel_error_status_flag

float32 vel_ratio # Velocity innovation test ratio
float32 pos_horiz_ratio # Horizontal position innovation test ratio
float32 pos_vert_ratio # Vertical position innovation test ratio
float32 mag_ratio # Magnetometer innovation test ratio
float32 hagl_ratio # Height above terrain innovation test ratio
float32 tas_ratio # True airspeed innovation test ratio
float32 pos_horiz_accuracy # [m] Horizontal position 1-STD accuracy relative to the EKF local origin
float32 pos_vert_accuracy # [m] Vertical position 1-STD accuracy relative to the EKF local origin
