# Header is the source image acquisition stamp, never inference completion time.
std_msgs/Header header
sensor_msgs/Image mask
sensor_msgs/CameraInfo camera_info
string calibration_id
string rig_calibration_sha256
string model_sha256
string preprocessing_revision
string[] class_names
# Mean softmax per predicted class; UNCALIBRATED. Empty classes have score 0.
float32[] mean_scores
builtin_interfaces/Duration processing_time
# Current implementation always false. Features are not insertion authorization.
bool motion_permitted
