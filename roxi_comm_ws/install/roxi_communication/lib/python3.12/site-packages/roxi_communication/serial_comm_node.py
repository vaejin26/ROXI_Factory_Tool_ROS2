import rclpy
from rclpy.node import Node
import serial.tools.list_ports
from std_msgs.msg import String
import json

class SerialCommNode(Node):
    def __init__(self):
        super().__init__('serial_comm_node')
        self.get_logger().info("Serial Communication Node Initialized")
        
        self.publisher_ = self.create_publisher(String, 'connected_ports', 10)
        self.timer = self.create_timer(1.0, self.publish_connected_ports)
        
        self.device_ports = {
            'MCU': '/dev/ttyUSB0',
            'TRAY': '/dev/ttyUSB1',
            'IMU': '/dev/ttyUSB2',
            'MOT': '/dev/ttyUSB3',
            'RP': '/dev/ttyUSB4',
        }
        
        self.connected_ports = {}
        self.find_and_connect_ports()
        
    def find_and_connect_ports(self):
        ports = serial.tools.list_ports.comports()
        available_ports = [port.device for port in ports]
        
        for name, excepted_port in self.device_ports.items():
            matched = next ((port for port in available_ports if excepted_port in port), None)
            
            if matched:
                self.get_logger().info(f"{name} connected on {matched}")
                try:
                    ser = serial.Serial(matched, 115200, timeout=1)
                    self.get_logger().info(f"Connected to {name} on Serial port {matched}")
                    ser.close()
                    self.connected_ports[name] = matched
                except Exception as e:
                    self.get_logger().error(f"Failed to connect to {name} on {matched}: {e}")
            else: 
                self.get_logger().warning(f"{name} not found on {excepted_port}")
                
    def publish_connected_ports(self):
        msg = String()
        msg.data = json.dumps(self.connected_ports)
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing connected ports: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialCommNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()