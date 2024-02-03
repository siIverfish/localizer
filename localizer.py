from typing import List
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray

class Localizer:
    def __call__(self, april_tag_detection_array):
        pos = april_tag_detection_array.detections[0].pose.pose.pose.position
        return [pos.x, pos.y, pos.z]
