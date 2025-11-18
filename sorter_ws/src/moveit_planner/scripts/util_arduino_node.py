import rclpy
from rclpy.node import Node 
from std_msgs.msg import String 
import serial 

port = '/dev/ttyACM0'  # Update this to your Arduino's serial port
baud_rate = 9600

class ArduinoNode(Node): 
    def __init__(self): 
        super().__init__('util_arduino_node') 
        self.publisher_ = self.create_subscription(String, 'arduino_cmds', self.command_callback, 10) 
        self.serial_port = serial.Serial(port, baud_rate, timeout=1)
    
    def command_callback(self, msg): 
        command = msg.data 
        self.get_logger().info(f'Received command: {command}') 
        self.serial_port.write((command + '\n').encode('utf-8')) 
    
def main(args=None): 
        rclpy.init(args=args) 
        arduino_node = ArduinoNode() 
        rclpy.spin(arduino_node) 
        arduino_node.destroy_node() 
        rclpy.shutdown() 
    
if __name__ == '__main__': 
    main()