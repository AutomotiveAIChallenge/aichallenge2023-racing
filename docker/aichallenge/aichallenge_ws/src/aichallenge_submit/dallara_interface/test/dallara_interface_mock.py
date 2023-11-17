import rclpy
from rclpy.node import Node
from autonoma_msgs.msg import PowertrainData, RaceControl, ToRaptor, VehicleData, VehicleInputs
from std_msgs.msg import Header

class DallaraInterfaceTestPublisher(Node):

    def __init__(self):
        super().__init__('dallara_interface_mock')
        self.count = 0

        self.create_subscription(PowertrainData, '/powertrain_data', self.powertrain_data_callback, 10)
        self.create_subscription(RaceControl, '/race_control', self.race_control_callback, 10)
        self.create_subscription(VehicleData, '/vehicle_data', self.vehicle_data_callback, 10)

        # Publishers for the messages
        self.to_raptor_publisher = self.create_publisher(ToRaptor, '/to_raptor', 10)
        self.vehicle_inputs_publisher = self.create_publisher(VehicleInputs, '/vehicle_inputs', 10)
        
        timer_period = 0.1  # 10Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def powertrain_data_callback(self, msg):
        self.get_logger().info('Received PowertrainData Message')
        print(msg)

    def race_control_callback(self, msg):
        self.get_logger().info('Received RaceControl Message')
        print(msg)

    def vehicle_data_callback(self, msg):
        self.get_logger().info('Received VehicleData Message')
        print(msg)

    def timer_callback(self):
        # Create and publish each message with default values
        header = Header(stamp=self.get_clock().now().to_msg())
        # Set values for ToRaptor message
        to_raptor = ToRaptor()
        to_raptor.header = header
        to_raptor.track_cond_ack = 0
        to_raptor.veh_sig_ack = 0
        to_raptor.ct_state = 1
        to_raptor.rolling_counter = 1
        to_raptor.veh_num = 1
        self.to_raptor_publisher.publish(to_raptor)
                
        vehicle_inputs = VehicleInputs()
        vehicle_inputs.throttle_cmd = 10.0
        vehicle_inputs.throttle_cmd_count = 1
        vehicle_inputs.brake_cmd = 0.0
        vehicle_inputs.brake_cmd_count = 1
        vehicle_inputs.steering_cmd = 10.0
        vehicle_inputs.steering_cmd_count = 1
        vehicle_inputs.gear_cmd = 2
        self.vehicle_inputs_publisher.publish(vehicle_inputs)

        self.get_logger().info('Messages Published')

def main(args=None):
    rclpy.init(args=args)
    msg_publisher = DallaraInterfaceTestPublisher()
    rclpy.spin(msg_publisher)
    msg_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
