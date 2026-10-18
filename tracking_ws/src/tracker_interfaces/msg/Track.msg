# One tracked object in image coordinates (pixels).
int32 id
string class_name
float32 score
float32 x1
float32 y1
float32 x2
float32 y2
# 0 = tentative (not yet confirmed), 1 = tracked (matched this frame), 2 = lost (coasting)
uint8 state
