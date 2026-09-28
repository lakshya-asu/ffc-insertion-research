# Contract only until a real or simulated sensor adapter is commissioned.
std_msgs/Header header
string calibration_id
uint32 rows
uint32 columns
float32[] normal_force_newtons
# Optional shear arrays must be empty when unmeasured, never invented zeros.
float32[] shear_x_newtons
float32[] shear_y_newtons
bool valid
