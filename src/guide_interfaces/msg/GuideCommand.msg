std_msgs/Header header

uint8 MODE_STOP=0
uint8 MODE_FORWARD=1
uint8 MODE_LEFT=2
uint8 MODE_RIGHT=3
uint8 MODE_CAUTION=4

uint8 command
float32 lateral_error_m
bool blind_path_lost
bool obstacle_near
uint8 obstacle_side
float32 confidence
