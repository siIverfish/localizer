#!/usr/bin/env python3

""" 
    This script will subscribe to the RoboRIO odometry feed;
    perform localization code; and publish final location of
    the robot on the field to NetworkTables so that 
    the RoboRIO can *know where it is!*
"""

import ntcore
import time

# wonky import, needs apt package
import rospy

from collections import deque
from typing import List

# todo: move these into their respective classes
TEAM_NUMBER = "3952"
CLIENT_NAME = "3952-odometry-subscriber"
TABLE_NAME = "datatable"
TOPIC_NAME  = "estimatedOdometryPosition"
NT_INSTANCE = ntcore.NetworkTableInstance.getDefault()
NT_TABLE = NT_INSTANCE.getTable(TABLE_NAME)
PUBLISH_TOPIC_NAME = "finalRobotPosition"

class OdometryPositionSubscriber:
    def __init__(self):
        self.nt_position_subscriber = NT_TABLE.getDoubleArrayTopic(TOPIC_NAME).subscribe(0)
        self.nt_instance.startClient4(CLIENT_NAME) # i do not know what this does
        self.nt_instance.setServerTeam(TEAM_NUMBER) # where TEAM=190, 294, etc, or use inst.setServer("hostname") or similar

    async def __iter__(self):
        # todo: make the `time.sleep` call here start counting the 1 second before the yield
        # instead of starting and stopping after with synchronous sleep
        last_result = None
        while True:
            current_result = self.nt_position_subscriber.get()
            if current_result != last_result:
                yield current_result
                last_result = current_result
            time.sleep(0.1)


class ApriltagPositionSubscriber:
    """
      Subscribes to the ROS topic (specified by ROS_TOPIC_NAME)
      and sends its data to the callback when received.
    """

    # https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_apriltag/isaac_ros_apriltag/index.html#quickstart
    ROS_TOPIC_NAME = "tag_detections"
    DATA_TYPE = "isaac_ros_apriltag_interfaces/AprilTagDetectionArray"
    NODE_NAME = "ApriltagPositionSubscriber@jetons_localization"


    def __init__(self, *, callback_function=None):
        # FILO queue for iteration
        self.queue = deque()

        rospy.init_node(self.NODE_NAME)
        rospy.Subscriber(self.ROS_TOPIC_NAME, self.DATA_TYPE, callback_function)
    
    
    async def callback(self, tag_detections):
        self.queue.appendleft(tag_detections)
    
    
    async def __iter__(self):
        while True:
            if self.queue:
                yield self.queue.pop()
            time.sleep(0.1)


class Localizer:
    def get_localized_position(self, apriltag_locations, odometry_location):
        ... # this code will be written after we decide on a localization library.
    
    def __call__(self, zipped_inputs):
        return self.get_localized_position(*zipped_inputs)


class NewPositionPublisher:
    # this class may be unnecessary
    def __init__(self):
        self._publisher = NT_TABLE.getDoubleArrayTopic(PUBLISH_TOPIC_NAME).publish()
    
    def publish_each(self, values):
        for value in values:
            self._publisher.set(values)


def main():
    odometry_position_subscriber = OdometryPositionSubscriber()
    apriltag_position_subscriber = ApriltagPositionSubscriber()
    localizer = Localizer()
    new_position_publisher = NewPositionPublisher()

    localization_inputs = zip(odometry_position_subscriber, apriltag_position_subscriber)
    final_positions = map(localizer, localization_inputs)
    new_position_publisher.publish_each(final_positions)


if __name__ == "__main__":
    main()