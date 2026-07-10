# Defines a YOLO detection result

# Detected class and confidence
uint16 class_id
string class_name
float32 confidence

# ID for tracking
uint16 id

# Bounding box
yolo_msgs/BoundingBox bbox

# Keypoints
yolo_msgs/Keypoint[] keypoints

# TODO: Segmentation mask
# yolo_msgs/Mask mask