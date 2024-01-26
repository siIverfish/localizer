class OdometryPositionSubscriber:
    NT_ODOMETRY_TOPIC_NAME  = "estimatedOdometryPosition"
    CLIENT_NAME = "3952-odometry-subscriber"
    TEAM_NUMBER = "3952"

    def __init__(self, *, table):
        self.nt_position_subscriber = table.getDoubleArrayTopic(self.NT_ODOMETRY_TOPIC_NAME).subscribe(0)
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