# About this Package
Convert Odometry to TF
# How To Use

1. run gtest for 10 sec

`colcon test --packages-select "odom2tf" --event-handlers console_direct+`

2. topic pub via terminal
`ros2 topic pub /localization/kinematic_state nav_msgs/msg/Odometry {}`

3. echo in terminal
`ros2 topic echo /tf`


# Input/Output

- input for gest in terminal
`ros2 topic pub /localization/kinematic_state nav_msgs/msg/Odometry {}`

```
publishing #277: nav_msgs.msg.Odometry(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id=''), child_frame_id='', pose=geometry_msgs.msg.PoseWithCovariance(pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=0.0, y=0.0, z=0.0), orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)), covariance=array([0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
       0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
       0., 0.])), twist=geometry_msgs.msg.TwistWithCovariance(twist=geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0)), covariance=array([0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
       0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
       0., 0.])))
```

- output in terminal

`ros2 topic echo /tf`

```
transforms:
- header:
    stamp:
      sec: 1695797937
      nanosec: 552073140
    frame_id: odom
  child_frame_id: base_link
  transform:
    translation:
      x: 0.0
      y: 0.0
      z: 0.0
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
---

```