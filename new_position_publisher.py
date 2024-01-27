from typing import List

class NewPositionPublisher:
    PUBLISH_TOPIC_NAME = "finalRobotPosition"

    def __init__(self, *, table) -> None:
        self._publisher = table\
            .getDoubleArrayTopic(self.PUBLISH_TOPIC_NAME)\
            .publish()
    
    def __call__(self, value: List[float]) -> None:
        self._publisher.set(value)