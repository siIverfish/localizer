"""
This class works similarly to the odometrey subscriber, but with different way of retrieving values. 
(See odometry_position_subscriber for an explanation of the iterator situation). 

To get a value, it has to hook into rclpy and run whatever subscriber code they have over there. Then, it checks if the hook caught any fish (if anything called the callback function) and only then does it get anything to return.

May have to alter this file to spin until it gets a new value.
"""

import rclpy

from apriltags_ros.msg import AprilTagDetectionArray

class AprilTagPositionSubscriber(rclpy.node.Node):
    """
      Subscribes to the ROS topic (specified by ROS_TOPIC_NAME)
      and sends its data to the callback when received.
    """

    ROS_TOPIC_NAME = "tag_detections"
    DATA_TYPE = AprilTagDetectionArray
    NODE_NAME = "ApriltagPositionSubscriber@jetons_localization"

    def __init__(self):
        super().__init__(self.NODE_NAME)
        self.subscription = self.create_subscription(
            self.DATA_TYPE,
            self.ROS_TOPIC_NAME,
            self.callback,
            # this is the history depth. may need to be set to 1
            # look at docs later
            10
        )


    def callback(self, tag_detections):
        self.value = tag_detections
    

    def __iter__(self):
        while True:
            rclpy.spin_once(self)
            if self.value:
                temp = self.value
                self.value = None
                yield temp
            # time.sleep(0.1) ~probably~ don't need this