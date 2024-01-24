#!/usr/bin/env python3

""" 
    This script will subscribe to the RoboRIO odometry feed;
    perform localization code; and publish final location of
    the robot on the field to NetworkTables so that 
    the RoboRIO can *know where it is!*
"""

import ntcore
import time

TEAM_NUMBER = "3952"
CLIENT_NAME = "3952-odometry-subscriber"
TABLE_NAME = "datatable"
TOPIC_NAME  = "estimatedOdometryPosition"
NT_INSTANCE = ntcore.NetworkTableInstance.getDefault()
NT_TABLE = NT_INSTANCE.getTable(TABLE_NAME)

class PositionSubscriber:
    def __init__(self, *, callback):
        self.callback = callback

        self.nt_position_subscriber = NT_TABLE.getDoubleArrayTopic(TOPIC_NAME).subscribe(0)
        self.nt_instance.startClient4(CLIENT_NAME) # i do not know what this does
        self.nt_instance.setServerTeam(TEAM_NUMBER) # where TEAM=190, 294, etc, or use inst.setServer("hostname") or similar

    def run(self):
        last_result = None
        while True:
            current_result = self.nt_position_subscriber.get()
            if current_result != last_result:
                self.callback(current_result)
                last_result = current_result
            time.sleep(1)


class NewPositionPublisher:
    def __init__(self, *, callback):

            
...
...
...
