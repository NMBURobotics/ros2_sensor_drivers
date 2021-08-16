# ros2_sensor_drivers
A collection of common sensors in robotics such as Ouster LIDAR, ZED Cam, Xsens IMU, nmea sentenced GPS(s) etc.

# Disclamer

Note that the packages have their own dependencies. For instance realsense and ZED both require their own SDKs and they needed to be installed for a successful build. Dependencies are not hosted in this package, only ROS2 interfaces of drivers are kept here for conveniently managing all sensors in one workspace.

You might be able to get most of required dependencies with; 

```bash
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy
```

However this wont satisfy all dependencies, refer to specific driver repositories for installing or source building. 