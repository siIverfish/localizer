"""
This class works similarly to the odometrey subscriber, but with different way of retrieving values. 
(See odometry_position_subscriber for an explanation of the iterator situation). 

To get a value, it has to hook into rclpy and run whatever subscriber code they have over there. 
Then, it checks if the hook caught any fish (if anything called the callback function) and only then does it get anything to return.

May have to alter this file to spin until it gets a new value. Spinning pile up past work and eventually be way backed up.

The problem with this code is that rclpy *really* wants to be a whole framework that encompasses all of our code, but it can't be --  we need other inputs too. 

This led me to think "wow! this code would turn out to be so easy if we did just that -- made a callback function that grabs odometry outputs itself and publishes them to network tables right there in the method." 
Then I realized I already *had* written code that did that, I was just dumb and didn't realize how my own iterators worked.
The `yield` statement in this function already triggers all of the rest of the iterator path, sending it down through the chain until eventually it is published.

thanks for listening to my tedtalk - ivan
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