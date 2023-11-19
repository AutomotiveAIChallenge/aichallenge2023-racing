
import rclpy
from rclpy.node import Node
from autonoma_msgs.msg import PowertrainData, RaceControl, ToRaptor, VehicleData, VehicleInputs
from std_msgs.msg import Header

class DallaraInterfaceTestSubscriber(Node):

    def __init__(self):
        super().__init__('awsim_mock')
        
        # Subscribers for the messages
        self.create_subscription(VehicleInputs, '/vehicle_inputs', self.vehicle_inputs_callback, 10)
        self.create_subscription(ToRaptor, '/to_raptor', self.to_raptor_callback, 10)

        # Publishers for the messages
        self.powertrain_data_publisher = self.create_publisher(PowertrainData, '/powertrain_data', 10)
        self.race_control_publisher = self.create_publisher(RaceControl, '/race_control', 10)
        self.vehicle_data_publisher = self.create_publisher(VehicleData, '/vehicle_data', 10)
        
        timer_period = 0.1  # 10Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        header = Header(stamp=self.get_clock().now().to_msg())
        # Create and publish each message with default values
        powertrain_data = PowertrainData()
        powertrain_data.header = header
        # Sensor data
        powertrain_data.map_sensor = 50.0
        powertrain_data.lambda_sensor = 0.9
        powertrain_data.fuel_level = 75.0
        powertrain_data.fuel_pressure = 300.0
        powertrain_data.engine_oil_pressure = 100.0
        powertrain_data.engine_oil_temperature = 90.0
        powertrain_data.engine_coolant_temperature = 85.0
        powertrain_data.engine_coolant_pressure = 110.0
        powertrain_data.engine_rpm = 3000.0
        powertrain_data.engine_on_status = True
        powertrain_data.engine_run_switch_status = True
        powertrain_data.throttle_position = 30.0
        powertrain_data.current_gear = 3
        powertrain_data.gear_shift_status = 0
        powertrain_data.transmission_oil_pressure = 120.0
        powertrain_data.transmission_accumulator_pressure = 115.0
        powertrain_data.transmission_oil_temperature = 80.0
        powertrain_data.vehicle_speed_kmph = 60.0
        powertrain_data.torque_wheels_nm = 250.0
        self.powertrain_data_publisher.publish(powertrain_data)
        
        race_control = RaceControl()
        # Set and publish RaceControl message
        race_control = RaceControl()
        race_control.header = header
        race_control.base_to_car_heartbeat = 1
        race_control.track_flag = 0
        race_control.veh_flag = 0
        race_control.veh_rank = 1
        race_control.lap_count = 5
        race_control.lap_distance = 1000.0
        race_control.round_target_speed = 100
        race_control.laps = 10
        race_control.lap_time = 60.0
        race_control.time_stamp = 300.0
        self.race_control_publisher.publish(race_control)
                
        vehicle_data = VehicleData()
        # Tire data
        vehicle_data.fl_tire_temperature = 25.0
        vehicle_data.fl_damper_linear_potentiometer = 0.1
        vehicle_data.fl_tire_pressure = 250.0
        vehicle_data.fl_tire_pressure_gauge = 250.0
        vehicle_data.fl_wheel_load = 1000.0
        vehicle_data.fr_tire_temperature = 25.0
        vehicle_data.fr_damper_linear_potentiometer = 0.1
        vehicle_data.fr_tire_pressure = 250.0
        vehicle_data.fr_tire_pressure_gauge = 250.0
        vehicle_data.fr_wheel_load = 1000.0
        vehicle_data.rl_tire_temperature = 25.0
        vehicle_data.rl_damper_linear_potentiometer = 0.1
        vehicle_data.rl_tire_pressure = 250.0
        vehicle_data.rl_tire_pressure_gauge = 250.0
        vehicle_data.rl_wheel_load = 1000.0
        vehicle_data.rr_tire_temperature = 25.0
        vehicle_data.rr_damper_linear_potentiometer = 0.1
        vehicle_data.rr_tire_pressure = 250.0
        vehicle_data.rr_tire_pressure_gauge = 250.0
        vehicle_data.rr_wheel_load = 1000.0
        
        # Brake temps
        vehicle_data.fl_brake_temp = 50.0
        vehicle_data.fr_brake_temp = 50.0
        vehicle_data.rl_brake_temp = 50.0
        vehicle_data.rr_brake_temp = 50.0
        
        # Misc data
        vehicle_data.battery_voltage = 12.0
        vehicle_data.safety_switch_state = 1
        vehicle_data.mode_switch_state = True
        vehicle_data.sys_state = 1
        
        # Accel pedal report
        vehicle_data.accel_pedal_input = 50.0
        vehicle_data.accel_pedal_output = 50.0
        
        # Brake report
        vehicle_data.front_brake_pressure = 100.0
        vehicle_data.rear_brake_pressure = 100.0
        
        # Steering Report
        vehicle_data.steering_wheel_angle = 0.0
        vehicle_data.steering_wheel_angle_cmd = 0.0
        vehicle_data.steering_wheel_torque = 50.0
        
        # Wheel speeds (kph)
        vehicle_data.ws_front_left = 50.0
        vehicle_data.ws_front_right = 50.0
        vehicle_data.ws_rear_left = 50.0
        vehicle_data.ws_rear_right = 50.0        # ... [Set other fields for VehicleData as needed]
        self.vehicle_data_publisher.publish(vehicle_data)
 
        self.get_logger().info('Messages Published')

    def to_raptor_callback(self, msg):
        self.get_logger().info('Received ToRaptor Message')
        print(msg)

    def vehicle_inputs_callback(self, msg):
        self.get_logger().info('Received VehicleInputs Message')
        print(msg)

def main(args=None):
    rclpy.init(args=args)
    msg_subscriber = DallaraInterfaceTestSubscriber()
    rclpy.spin(msg_subscriber)
    msg_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
