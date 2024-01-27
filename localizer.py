from typing import List
from apriltags_ros.msg import AprilTagDetectionArray

class Localizer:
    def get_localized_position(
            self, 
            apriltag_locations: AprilTagDetectionArray, 
            odometry_location: List[float],
            ) -> List[float]:
        raise NotImplementedError() # this code will be written after we decide on a localization library.
    
    def __call__(self, zipped_inputs):
        return self.get_localized_position(*zipped_inputs)
