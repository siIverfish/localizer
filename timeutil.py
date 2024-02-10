import time

from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray

def timestamp(april_tag_detection_array: AprilTagDetectionArray):
    """ Returns time (in seconds) since epoch apriltags were detected, w/ fractional part."""
    return april_tag_detection_array.detections[0].pose.header.stamp.sec + \
           april_tag_detection_array.detections[0].pose.header.stamp.nanosec / 1e9

def is_recent(april_tag_detection_array: AprilTagDetectionArray, *, seconds):
    current_time_secs = time.time()
    detection_time_secs = timestamp(april_tag_detection_array)
    time_since_detected_secs = detection_time_secs
    print("Latency (AprilTagPositionSubscriber):", time_since_detected_secs)
    return time_since_detected_secs < seconds