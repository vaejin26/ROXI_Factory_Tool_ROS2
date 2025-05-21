import rclpy
from rclpy.node import Node
import serial
from roxi_communication.serial_comm_node import SerialCommNode
import struct

class IMUCommNode(Node):
    def __init__(self):
        super().__init__('imu_comm_node')
        
        self.serial_helper = SerialCommNode()
        imu_port = self.serial_helper.connected_ports.get('IMU')
        
        if not imu_port :
            self.get_logger().error("Cannot find IMU port")
            return
        
        try:
            self.ser = serial.Serial(imu_port, baudrate=115200, timeout=1)
            self.get_logger().info(f"IMU connected: {imu_port}")
            
            self.read_IMU_data()
            
        except Exception as e:
            self.get_logger().error(f"Cannot connect to IMU: {e}")
            
    def read_IMU_data(self):
        while True:
            header = self.ser.read(1)
            if header != b'\x55':
                continue
            
            data = self.ser.read(10)
            if len(data) != 10:
                continue
            
            full_packet = b'\x55' + data
            packet_type = data[0]
            
            if packet_type == 0x51:
                ax, ay, az = self.parse_sensor_packet(data[1:7], 16.0)
                self.get_logger().info(f"Accel : ax={ax:.2f}, ay={ay:.2f}, az={az:.2f}")
            elif packet_type == 0x52:
                wx, wy, wz = self.parse_sensor_packet(data[1:7], 2000.0)
                self.get_logger().info(f"Gyro  : wx={wx:.2f}, wy={wy:.2f}, wz={wz:.2f}")
            elif packet_type == 0x53:
                roll, pitch, yaw = self.parse_sensor_packet(data[1:7], 180.0)
                self.get_logger().info(f"Angle : roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}")
                
    def parse_sensor_packet(self, raw_bytes, scale):
        x_raw = struct.unpack('<h', raw_bytes[0:2])[0]
        y_raw = struct.unpack('<h', raw_bytes[2:4])[0]
        z_raw = struct.unpack('<h', raw_bytes[4:6])[0]
        x = x_raw / 32768.0 * scale
        y = y_raw / 32768.0 * scale
        z = z_raw / 32768.0 * scale
        return x, y, z
    
def main(args=None):
    rclpy.init(args=args)
    node = IMUCommNode()
    rclpy.spin_once(node, timeout_sec=2.0)
    node.destroy_node()
    rclpy.shutdown()