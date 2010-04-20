uint16 POSITION=1
uint16 ORIENTATION=2
uint16 LOOK_AT=4
uint16 FOVY=8
uint16 ASPECT_RATIO=16
uint16 AUTO_ASPECT_RATIO=32
uint16 NEAR_CLIP=64
uint16 FAR_CLIP=128
uint16 MOVE=256
uint16 ROTATE=512

uint16 flags
UUID id

geometry_msgs/Vector3 position
geometry_msgs/Quaternion orientation
geometry_msgs/Vector3 look_at
float32 fovy
float32 aspect_ratio
bool auto_aspect_ratio
float32 near_clip
float32 far_clip
CameraMoveCommand move
geometry_msgs/Quaternion rotate