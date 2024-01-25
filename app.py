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

TABLE_NAME = "datatable"
NT_INSTANCE = ntcore.NetworkTableInstance.getDefault()
NT_TABLE = NT_INSTANCE.getTable(TABLE_NAME)


def main():
    odometry_position_subscriber = OdometryPositionSubscriber()
    apriltag_position_subscriber = ApriltagPositionSubscriber()
    localizer = Localizer()
    new_position_publisher = NewPositionPublisher()

    localization_inputs = zip(odometry_position_subscriber, apriltag_position_subscriber)
    final_positions = map(localizer, localization_inputs)
    new_position_publisher.publish_each(final_positions)


class OdometryPositionSubscriber:
    NT_ODOMETRY_TOPIC_NAME  = "estimatedOdometryPosition"
    CLIENT_NAME = "3952-odometry-subscriber"
    TEAM_NUMBER = "3952"

    def __init__(self, *, table):
        self.nt_position_subscriber = NT_TABLE.getDoubleArrayTopic(self.NT_ODOMETRY_TOPIC_NAME).subscribe(0)
        # these values are probably mostly diagnostic
        self.nt_instance.startClient4(self.CLIENT_NAME) # i do not know what this does
        self.nt_instance.setServerTeam(self.TEAM_NUMBER) # where TEAM=190, 294, etc, or use inst.setServer("hostname") or similar

    async def __iter__(self):
        # todo: make the `time.sleep` call here start counting the 1 second before the yield
        # instead of starting and stopping after with async sleep
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
    # this might be the wrong topic name, could be "tag_detections"
    ROS_TOPIC_NAME = "tag_detections"
    DATA_TYPE = "sensor_msgs/CameraInfo"
    NODE_NAME = "ApriltagPositionSubscriber@jetons_localization"


    def __init__(self, *, callback_function=None):
        # Optional most recent value, returned by the next() method.
        # if the next() method is called while this is None,
        # it waits until it receives a new value from isaac_ros to return.
        self.value = None

        rospy.init_node(self.NODE_NAME)
        rospy.Subscriber(self.ROS_TOPIC_NAME, self.DATA_TYPE, callback_function)
    
    
    async def callback(self, tag_detections):
        self.value = tag_detections
    
    
    async def __iter__(self):
        while True:
            if self.value:
                # this could be better, find an Optional<> datatype
                # like Rust has & import it or something.
                temp = self.value
                self.value = None
                yield temp
            time.sleep(0.1)


class Localizer:
    def get_localized_position(self, apriltag_locations, odometry_location):
        ... # this code will be written after we decide on a localization library.
    
    def __call__(self, zipped_inputs):
        return self.get_localized_position(*zipped_inputs)


class NewPositionPublisher:
    PUBLISH_TOPIC_NAME = "finalRobotPosition"

    def __init__(self, *, table):
        self._publisher = table\
            .getDoubleArrayTopic(self.PUBLISH_TOPIC_NAME)\
            .publish()
    
    def publish_each(self, values):
        for value in values:
            self._publisher.set(values)


if __name__ == "__main__":
    main()