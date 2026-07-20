# Defines a YOLO detection result

# Tracking ID used when no tracker has assigned an ID
uint16 UNCHECKED=65535

# Detected class and confidence
uint16 class_id
string class_name
float32 confidence

# Tracking ID
uint16 id

# Bounding box
yolo_msgs/BoundingBox bbox

# Keypoints
yolo_msgs/Keypoint[] keypoints

# TODO: Segmentation mask
# yolo_msgs/Mask mask
