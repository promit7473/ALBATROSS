# Flight / follow controller status for the operator console.
std_msgs/Header header
string flight_state        # DISCONNECTED, LANDED, TAKEOFF, HOVER, FOLLOW, SEARCH, LANDING
string follow_mode         # behind | hold_bearing | orbit
bool connected
bool armed
bool offboard
geometry_msgs/Point drone_position      # local ENU [m]
float32 drone_yaw                       # [rad]
bool target_valid
geometry_msgs/Point target_position     # estimated target ENU [m]
geometry_msgs/Vector3 target_velocity   # [m/s]
float32 target_distance                 # horizontal drone-target distance [m]
float32 desired_distance
float32 desired_altitude
