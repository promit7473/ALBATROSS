# The operator-selected (locked) target, as seen by perception.
uint8 NONE = 0       # nothing locked
uint8 TRACKING = 1   # locked target visible this frame
uint8 LOST = 2       # locked target not visible; searching / re-identifying

std_msgs/Header header
uint8 status
int32 track_id       # current track id (may change after re-identification)
Track track          # last known box (valid when status != NONE)
float32 lost_for     # seconds since last seen
uint32 image_width
uint32 image_height
