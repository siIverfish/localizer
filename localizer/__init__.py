#!/usr/bin/env python3

""" 
    This script will subscribe to the RoboRIO odometry feed;
    perform localization code; and publish final location of
    the robot on the field to NetworkTables so that 
    the RoboRIO can *know where it is!*

    Uses piped iterators for all processing.
"""

import ntcore
import rclpy

from localizer.odometry_position_subscriber import OdometryPositionSubscriber
from localizer.apriltag_position_subscriber import AprilTagPositionSubscriber
from localizer.new_position_publisher import NewPositionPublisher
from localizer.localizer import Localizer

TABLE_NAME = "datatable"
NT_INSTANCE = ntcore.NetworkTableInstance.getDefault()
NT_TABLE = NT_INSTANCE.getTable(TABLE_NAME)


def main(args=None):
    rclpy.init(args=args)

    # Set up inputs, localizer, and output
    odometry_position_subscriber = OdometryPositionSubscriber(NT_TABLE)
    apriltag_position_subscriber = AprilTagPositionSubscriber()
    localizer = Localizer()
    new_position_publisher = NewPositionPublisher(NT_TABLE)

    # setup iterator flow path & go
    localization_inputs = zip(odometry_position_subscriber, apriltag_position_subscriber)
    final_positions = map(localizer, localization_inputs)
    new_position_publisher.publish_each(final_positions)

    # this code won't happen, but i'll leave it in case a 
    # StopIteration case is added to `new_position_publisher.publish_each` 
    rclpy.shutdown()


if __name__ == "__main__":
    main()