
class Localizer:
    def get_localized_position(self, apriltag_locations, odometry_location):
        ... # this code will be written after we decide on a localization library.
    
    def __call__(self, zipped_inputs):
        return self.get_localized_position(*zipped_inputs)
