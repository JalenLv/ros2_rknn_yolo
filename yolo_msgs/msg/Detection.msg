# Defines a YOLO detection result

# Tracking ID used when no tracker has assigned an ID
uint32 UNCHECKED=4294967295

# Detected class and confidence
uint16 class_id
string class_name
float32 confidence

# Tracking ID
uint32 id

# Bounding box
yolo_msgs/BoundingBox bbox

# Keypoints
yolo_msgs/Keypoint[] keypoints

# ReID feature embedding (empty = absent)
float32[] embedding

# TODO: Segmentation mask
# yolo_msgs/Mask mask
