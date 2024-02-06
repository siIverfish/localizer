from typing import List
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray

from itertools import chain

# no longer localizes. now just extracts april tag positions.
# rip bad name, will eventually fix

class Localizer:
    def __call__(self, april_tag_detection_array):
        concatenated_positions = []
        for detection in april_tag_detection_array.detections:
            position = detection.pose.pose.pose.position
            concatenated_positions.extend((position.x, position.y, position.z))
        return concatenated_positions
