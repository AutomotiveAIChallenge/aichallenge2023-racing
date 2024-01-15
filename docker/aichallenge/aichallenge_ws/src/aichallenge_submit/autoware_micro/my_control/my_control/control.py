import rclpy
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autonoma_msgs.msg import VehicleInputs
from autoware_auto_vehicle_msgs.msg import VelocityReport

import numpy as np

def modified_throttle_function_with_target(velocity, target_velocity):
    """
    Modified throttle function considering the target velocity.
    Args:
    - velocity (float): The current velocity in m/s.
    - target_velocity (float): The target velocity in m/s.

    Returns:
    - throttle (float): The throttle value (0 to 100).
    """
    velocity_difference = target_velocity - velocity
    if velocity_difference > 0:
        # Accelerate more if the difference is larger
        throttle = min(100, velocity_difference * 10 + 2)
    else:
        # No acceleration if current velocity is at or above target
        throttle = 0

    return float(throttle)

def modified_brake_function_with_target(velocity, target_velocity):
    """
    Modified brake function considering the target velocity.
    Args:
    - velocity (float): The current velocity in m/s.
    - target_velocity (float): The target velocity in m/s.

    Returns:
    - brake (float): The brake force (0 to 100).
    """
    velocity_difference = velocity - target_velocity
    if velocity_difference > 0:
        # Brake more if the difference is larger
        brake = min(30, velocity_difference * 2.0)
    else:
        # No braking if current velocity is at or below target
        brake = 0.0

    return float(brake*100)


def shift_down(speed, speed_map, current_gear):
    return current_gear    


def shift_up(speed,gear,speed_map):
    return gear

    
def gear_change(speed,current_velocity,accel,brake,gear):
    #
    speed_map = [0.0, 50.0, 100.0, 150, 200, 250, 300] 
    calculated_gear = 0
    current_gear = gear
    if current_velocity > speed:
        calculated_gear = shift_down(speed,speed_map,current_gear)
        
        
    elif current_velocity < speed: 
        calculated_gear = shift_up(speed,gear,speed_map)

    calculated_gear = current_gear
    
    return  calculated_gear



class Controller(Node):
    def __init__(self):
        super().__init__('py_path_create_node')

        self.gear = 0
        self.speed = 0.0
        self.accel = 0.0
        self.brake = 0.0
        self.current_velocity = 0.0


        self.sub_pure_cmd = self.create_subscription(
            AckermannControlCommand,
            'input_pure_cmd',
            self.pure_cmd_callback,
            1)
        
        self.sub_dallara_cmd = self.create_subscription(
            VehicleInputs,
            'input_dallara_cmd',
            self.dallara_cmd_callback,
            1)
        
        self.sub_velocity_cmd = self.create_subscription(
            VelocityReport,
            'input_velocity_cmd',
            self.velocity_cmd_callback,
            1)
        
        
    
        self.pub_pure_cmd = self.create_publisher(
            AckermannControlCommand,
            'output_pure_cmd',
            1)
        
        self.pub_dallara_cmd = self.create_publisher(
            VehicleInputs,
            'output_dallara_cmd',
            1)
        
        self.pub_velocity_cmd = self.create_publisher(
            VelocityReport,
            'output_velocity_cmd',
            1)

    def pure_cmd_callback(self, msg):
        self.speed = msg.longitudinal.speed
        
        self.pub_pure_cmd.publish(msg)

    def dallara_cmd_callback(self, msg):
        dallara_msg = msg
        self.gear = dallara_msg.gear_cmd
        self.accel = dallara_msg.throttle_cmd
        self.brake = dallara_msg.brake_cmd

        self.gear = gear_change(self.speed,self.current_velocity, self.accel,self.brake,self.gear)
        
        # if self.current_velocity > self.speed:
        #     self.brake = brake_function(self.current_velocity)
        #     print(self.brake)
        # elif self.current_velocity < self.speed:
        #     self.accel = throttle_function(self.current_velocity)
           
        self.accel = modified_throttle_function_with_target(self.current_velocity,self.speed)
        self.brake = modified_brake_function_with_target(self.current_velocity,self.speed)
        dallara_msg.gear_cmd = self.gear
        dallara_msg.throttle_cmd = self.accel
        dallara_msg.brake_cmd = self.brake
        
        # print(dallara_msg)
        self.pub_dallara_cmd.publish(dallara_msg)

    def velocity_cmd_callback(self, msg):
        self.current_velocity = msg.longitudinal_velocity
        
        self.pub_velocity_cmd.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
