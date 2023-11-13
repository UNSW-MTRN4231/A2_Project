import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class ArduinoNode(Node):
    def __init__(self):
        super().__init__('util_arduino_serial')
        self.subscription = self.create_subscription(String, 'arduino_command', self.command_callback, 10)
        self.serial_port = serial.Serial('/dev/ttyACM0', 9600)  # Update the baud rate as required
        self.timer = self.create_timer(1,self.timer_callback)
        self.prevCommand = ""
        print("test")

    def command_callback(self, msg):
        command = msg.data
        if (self.prevCommand != command):
            self.get_logger().info(f"Sending command: {command}")
            self.serial_port.write(command.encode('utf-8'))
        self.prevCommand = command

    def timer_callback(self):
        received = self.serial_port.read_all()
        self.get_logger().info(f"Arduino: {received}")
        if (received == "b'Setting up Arduino\r\n'"):
            self.prevCommand = ""

def main(args=None):
    rclpy.init(args=args)
    util_arduino_serial = ArduinoNode()
    rclpy.spin(util_arduino_serial)
    util_arduino_serial.destroy_node()
    rclpy.shutdown()
    print("shutdown")

if __name__ == '__main__':
    main()
