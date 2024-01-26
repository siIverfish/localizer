class NewPositionPublisher:
    PUBLISH_TOPIC_NAME = "finalRobotPosition"

    def __init__(self, *, table):
        self._publisher = table\
            .getDoubleArrayTopic(self.PUBLISH_TOPIC_NAME)\
            .publish()
    
    def __call__(self, value):
        self._publisher.set(value)