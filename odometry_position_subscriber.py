"""
This class (and the AprilTagPositionSubscriber)  *seem* simple, but actually do not work very well as traditional iterators. 
The assumption about this iterator is that every time the robot publishes a value, it is collected in a queue and eventually fed out via the next method.
However, this class doesn't have a queue: rather, there is a single `last_result` value updated whenever the loop runs. 
If you read this as normal, non-generator code, it seems like it would continuously update and output values.
But it is not. Because of the gap after each yield statement as the localizer runs, it will *skip multiple values* that are never retrieved. In fact, when the next method is called, it will:
    a. return the value it gets polling network tables because it recognizes that it's a new value.
    b. poll and get the same value as last time (should be rarer, might never happen) in which case it'll re-poll until it gets something new.

Basically, the point is that the next() method will *always* return the most recent value. This circumvents problems with the iterators getting out of sync as one goes faster than the other.
"""

class OdometryPositionSubscriber:
    NT_ODOMETRY_TOPIC_NAME  = "estimatedOdometryPosition"
    CLIENT_NAME = "3952-odometry-subscriber"
    TEAM_NUMBER = "3952"

    def __init__(self, *, table):
        self.nt_position_subscriber = table.getDoubleArrayTopic(self.NT_ODOMETRY_TOPIC_NAME).subscribe([0.0])
        # these values are probably mostly diagnostic
        self.nt_instance.startClient4(self.CLIENT_NAME) # i do not know what this does
        self.nt_instance.setServerTeam(self.TEAM_NUMBER) # where TEAM=190, 294, etc, or use inst.setServer("hostname") or similar

    def __iter__(self):
        # todo: make the `time.sleep` call here start counting the 1 second before the yield
        # instead of starting and stopping after with synchronous sleep
        last_result = None
        while True:
            current_result = self.nt_position_subscriber.get()
            if current_result != last_result:
                yield current_result
                last_result = current_result
            # time.sleep(0.1) this ~probably~ isn't needed