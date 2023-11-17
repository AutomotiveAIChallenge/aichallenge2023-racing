
import rclpy
from rclpy.node import Node
from autoware_auto_planning_msgs.msg import Path, PathWithLaneId, Trajectory, PathPoint, PathPointWithLaneId, TrajectoryPoint
from autoware_auto_control_msgs.msg import AckermannControlCommand, AckermannLateralCommand, HighLevelControlCommand, LongitudinalCommand
from autoware_auto_vehicle_msgs.msg import GearCommand, GearReport, SteeringReport, VelocityReport

from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point
from builtin_interfaces.msg import Duration
from builtin_interfaces.msg import Time


class PathPublisher(Node):

    def __init__(self):
        super().__init__('autoware_mock')
        self.path_publisher = self.create_publisher(Path, 'path_topic', 10)
        self.path_with_lane_id_publisher = self.create_publisher(PathWithLaneId, 'path_with_lane_id_topic', 10)
        self.trajectory_publisher = self.create_publisher(Trajectory, '/planning/trajectory', 10)
        # Publishers for the messages
        self.ackermann_control_command_publisher = self.create_publisher(AckermannControlCommand, '/control/command/control_cmd', 10)
        self.gear_command_publisher = self.create_publisher(GearCommand, '/control/command/gear_cmd', 10)
        
        timer_period = 0.1  # 10Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Create messages
        header = Header()
        header.frame_id = "map"
        
        # Path message
        path_msg = Path()
        path_msg.header = header
        for i in range(11):
            point = PathPoint()
            point.pose.position.x = float(i)
            point.pose.position.y = 0.0
            path_msg.points.append(point)
        path_msg.left_bound = [Point(x=float(i), y=-float(1)) for i in range(11)]
        path_msg.right_bound = [Point(x=float(i), y=float(1)) for i in range(11)]
        
        # PathWithLaneId message
        path_with_lane_id_msg = PathWithLaneId()
        path_with_lane_id_msg.header = header
        for i in range(11):
            point_with_lane_id = PathPointWithLaneId()
            point_with_lane_id.point.pose.position.x = float(i)
            point_with_lane_id.point.pose.position.y = 0.0
            path_with_lane_id_msg.points.append(point_with_lane_id)
        path_with_lane_id_msg.left_bound = [Point(x=float(i), y=-float(1)) for i in range(11)]
        path_with_lane_id_msg.right_bound = [Point(x=float(i), y=float(1)) for i in range(11)]
        
        # Trajectory message
        trajectory_msg = Trajectory()
        trajectory_msg.header = header
        for i in range(11):
            traj_point = TrajectoryPoint()
            traj_point.time_from_start = Duration(sec=i)
            traj_point.pose.position.x = float(i)
            traj_point.pose.position.y = 0.0
            trajectory_msg.points.append(traj_point)
        
        # Publish messages
        self.path_publisher.publish(path_msg)
        self.path_with_lane_id_publisher.publish(path_with_lane_id_msg)
        self.trajectory_publisher.publish(trajectory_msg)
        self.get_logger().info('Messages Published')

        # Create and publish each message
        timestamp = Time(sec=int(self.get_clock().now().seconds_nanoseconds()[0]))
        ackermann_control_cmd = AckermannControlCommand()
        ackermann_control_cmd.stamp = timestamp
        self.ackermann_control_command_publisher.publish(ackermann_control_cmd)
         
        gear_cmd = GearCommand()
        gear_cmd.stamp = timestamp
        gear_cmd.command = 1
        self.gear_command_publisher.publish(gear_cmd)


def main(args=None):
    rclpy.init(args=args)
    path_publisher = PathPublisher()
    rclpy.spin(path_publisher)
    path_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()