import sys
import rclpy
from PyQt5 import QtCore, QtGui
from PyQt5.QtWidgets import QMainWindow, QVBoxLayout, QLabel, QSlider, QDial, QApplication, QComboBox, QPushButton, QTextEdit, QGraphicsOpacityEffect
from rclpy.node import Node
from PyQt5 import QtWidgets
from autonoma_msgs.msg import VehicleInputs, VehicleData

class RaceControlWidget(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setupUI()

    def setupUI(self):
        self.setObjectName("RaceControlMainWindow")
        self.resize(480, 500)
        self.setWindowTitle("Race Control Dashboard")

        # Set dark theme palette
        dark_palette = QtGui.QPalette()
        dark_palette.setColor(QtGui.QPalette.Window, QtGui.QColor(53, 53, 53))
        dark_palette.setColor(QtGui.QPalette.WindowText, QtCore.Qt.white)
        dark_palette.setColor(QtGui.QPalette.Base, QtGui.QColor(25, 25, 25))
        dark_palette.setColor(QtGui.QPalette.AlternateBase, QtGui.QColor(53, 53, 53))
        dark_palette.setColor(QtGui.QPalette.Text, QtCore.Qt.white)
        dark_palette.setColor(QtGui.QPalette.Button, QtGui.QColor(53, 53, 53))
        dark_palette.setColor(QtGui.QPalette.ButtonText, QtCore.Qt.white)
        self.setPalette(dark_palette)

        self.centralwidget = QtWidgets.QWidget(self)
        self.centralwidget.setObjectName("centralwidget")
        self.layout = QtWidgets.QHBoxLayout(self.centralwidget)

        # Steering Group (using QDial for steering)
        self.steering_group = QtWidgets.QGroupBox("Steering:")
        self.steering_layout = QtWidgets.QVBoxLayout()
        self.steering_cmd_dial = QtWidgets.QDial()  # Using QDial for steering
        self.steering_cmd_dial.setRange(-35, 35)
        self.steering_layout.addWidget(self.steering_cmd_dial)
        self.steering_group.setLayout(self.steering_layout)
        self.layout.addWidget(self.steering_group)

        # Throttle Group (range set to 1-9000)
        self.throttle_group = QtWidgets.QGroupBox("Throttle:")
        self.throttle_layout = QtWidgets.QVBoxLayout()
        self.throttle_cmd_slider = QtWidgets.QSlider(QtCore.Qt.Vertical)
        self.throttle_cmd_slider.setRange(1, 9000)
        self.throttle_layout.addWidget(self.throttle_cmd_slider)
        self.throttle_group.setLayout(self.throttle_layout)
        self.layout.addWidget(self.throttle_group)

        # Brake Group
        self.brake_group = QtWidgets.QGroupBox("Brake:")
        self.brake_layout = QtWidgets.QVBoxLayout()
        self.brake_cmd_slider = QtWidgets.QSlider(QtCore.Qt.Vertical)
        self.brake_layout.addWidget(self.brake_cmd_slider)
        self.brake_group.setLayout(self.brake_layout)
        self.layout.addWidget(self.brake_group)

        # ... (continue with the rest of your UI setup)

        self.centralwidget.setLayout(self.layout)
        self.setCentralWidget(self.centralwidget)

        # Gear slot UI
        self.gear_slot = QComboBox()
        self.gear_slot.addItems(['1', '2', '3', '4', '5', '6'])
        self.layout.addWidget(QLabel("Gear:"))
        self.layout.addWidget(self.gear_slot)

        # Submit button to send the commands
        self.submit_button = QPushButton("Send VehicleInputs")
        self.layout.addWidget(self.submit_button)

        # VehicleData Display
        self.vehicle_data_display = QTextEdit()
        self.vehicle_data_display.setReadOnly(True)
        self.layout.addWidget(QLabel("Received VehicleData:"))
        self.layout.addWidget(self.vehicle_data_display)

        self.centralwidget.setLayout(self.layout)
        self.setCentralWidget(self.centralwidget)


        # Adjusting the window appearance for aesthetics

class RaceControlNode(Node):
    def __init__(self):
        super().__init__("RaceControlNode")
        
        # Qt
        self.widget = RaceControlWidget()
        self.widget.show()
        self.widget.submit_button.clicked.connect(self.send_vehicle_inputs)

        # ROS
        self.vehicle_inputs_publisher = self.create_publisher(VehicleInputs, '/vehicle_inputs', 10)
        self.create_subscription(VehicleData, '/vehicle_data', self.vehicle_data_callback, 10)
       # Creating a timer to constantly send vehicle inputs
        self.timer = self.create_timer(0.1, self.send_vehicle_inputs)  # Sending vehicle inputs every 0.1 seconds

    def send_vehicle_inputs(self):
        vehicle_inputs = VehicleInputs()
        vehicle_inputs.throttle_cmd = float(self.widget.throttle_cmd_slider.value())
        vehicle_inputs.brake_cmd = self.widget.brake_cmd_slider.value() / 100.0
        vehicle_inputs.steering_cmd = -float(self.widget.steering_cmd_dial.value())
        vehicle_inputs.gear_cmd = int(self.widget.gear_slot.currentIndex())
        self.vehicle_inputs_publisher.publish(vehicle_inputs)

    def vehicle_data_callback(self, msg):
        self.widget.vehicle_data_display.setPlainText(str(msg))

def main(args=None):
    app = QApplication(sys.argv)

    rclpy.init(args=args)
    node = RaceControlNode()

    while True:
        app.processEvents()
        rclpy.spin_once(node, timeout_sec=0.01)

if __name__ == '__main__':
    main()