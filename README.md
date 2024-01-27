# jetons_localization

This repository passes values through an assembly line of classes: `AprilTagPositionSubscriber`, which gets values from our `isaac_ros_apriltag` ROS node; `OdometryPositionSubscriber`, which gets values from Avni's odometry publisher in the RoboRIO; `Localizer`, which performs localization with the streamed values; and finally `NewPositionPublisher`, which publishes these values to the RoboRIO.


This code doesn't work -- it's still being integrated into the `rclpy` ROS framework. Once someone gets in functional, they should push the entire `$ROS_WS/src/localizer` folder. Also, we need to actually finish `Localizer` with a localization library.


See individual files for more information.
