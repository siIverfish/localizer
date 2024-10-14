
from typing import List
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
from .timeutil import timestamp

# no longer localizes. now just extracts april tag positions.
# rip bad name, will eventually fix

class ROSNTTranslator:
    """ 
        Takes in an `AprilTagDetectionArray` and outputs a `List[float]`

        concatenated_positions[0] is always the timestamp.
        concatenated_positions[1-7] are the position/rotation of the first apriltag,
        in this format:
            position.x,
            position.y,
            position.z,
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w
        concatenated_positions[8-14] are the second apriltag, and so on.
    """

    def __call__(self, april_tag_detection_array: AprilTagDetectionArray) -> List[float]:
        concatenated_positions: List[float] = []
        concatenated_positions.append( timestamp(april_tag_detection_array) )

        for detection in april_tag_detection_array.detections:
            position = detection.pose.pose.pose.position
            rotation = detection.pose.pose.pose.orientation
            concatenated_positions.extend((
                position.x,
                position.y,
                position.z,
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w,
            ))

        return concatenated_positions
