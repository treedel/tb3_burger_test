# tb3_burger_test

Testing ros2 implementation of a turtlebot3-burger based robot.

## Todo
- [x] Obtain robot description
- [x] Implement basic hardware control with ESP32
- [ ] Test basic hardware functionality
- [x] Implement PID Control and fine tune its constants
- [x] Integrate ros2_control with hardware
- [ ] Test basic functionality with teleop_keyboard
- [ ] Attach IMU with PI and make it properly publish IMU msgs
- [ ] Apply sensor fusion to fuse IMU and Odometry
- [ ] Test basic navigation without LIDAR by using static tf between odom and map
- [ ] Attach LIDAR with PI and make it publish LaserScan msgs
- [ ] Test final functionality
- [ ] Debug and resolve any leftover issues

## References
- https://github.com/Slamtec/sllidar_ros2
- https://github.com/kimsniper/ros2_mpu6050
- https://www.sparkfun.com/hobby-motor-with-encoder-metal-gear-dg01d-e.html
- https://emanual.robotis.com/docs/en/platform/turtlebot3/features/
