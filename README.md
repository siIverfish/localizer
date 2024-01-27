# jetons_localization

This repository passes values through an assembly line of classes: `AprilTagPositionSubscriber`, which gets values from our `isaac_ros_apriltag` ROS node; `OdometryPositionSubscriber`, which gets values from Avni's odometry publisher in the RoboRIO; Localizer, which performs localization with the streamed values; and finally NewPositionPublisher, whose purpose is self-apparent.
