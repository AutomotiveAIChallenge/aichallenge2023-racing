std_msgs/Header                 header

# Tire data
float32 fl_tire_temperature # [deg C]
float32 fl_damper_linear_potentiometer # [m]
float32 fl_tire_pressure # [mbar]
float32 fl_tire_pressure_gauge # [mbar]
float32 fl_wheel_load # [N]
float32 fr_tire_temperature # [deg C]
float32 fr_damper_linear_potentiometer # [m]
float32 fr_tire_pressure # [mbar]
float32 fr_tire_pressure_gauge # [mbar]
float32 fr_wheel_load # [N]
float32 rl_tire_temperature # [deg C]
float32 rl_damper_linear_potentiometer # [m]
float32 rl_tire_pressure # [mbar]
float32 rl_tire_pressure_gauge # [mbar]
float32 rl_wheel_load # [N]
float32 rr_tire_temperature # [deg C]
float32 rr_damper_linear_potentiometer # [m]
float32 rr_tire_pressure # [mbar]
float32 rr_tire_pressure_gauge # [mbar]
float32 rr_wheel_load # [N]

# Brake temps
float32 fl_brake_temp # [deg C]
float32 fr_brake_temp # [deg C]
float32 rl_brake_temp # [deg C]
float32 rr_brake_temp # [deg C]

# Misc data
float32 battery_voltage
uint8 safety_switch_state
bool mode_switch_state
uint8 sys_state

# Accel pedal report 
float32 accel_pedal_input # [%] 0 to 100%
float32 accel_pedal_output # [%] 0 to 100%

# Brake report
float32 front_brake_pressure # [kPa]
float32 rear_brake_pressure # [kPa]

# Steering Report
float32 steering_wheel_angle      # [deg] motor angle
float32 steering_wheel_angle_cmd  # [deg] motor angle
float32 steering_wheel_torque     # 0 to 100%

# Wheel speeds (kph)
float32 ws_front_left # [km/h]
float32 ws_front_right # [km/h]
float32 ws_rear_left # [km/h]
float32 ws_rear_right # [km/h]