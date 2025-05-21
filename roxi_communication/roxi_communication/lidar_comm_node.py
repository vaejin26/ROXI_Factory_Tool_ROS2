import rclpy
from rclpy.node import Node
import serial
from roxi_communication.serial_comm_node import SerialCommNode
from roxi_communication.comm_protocol import (
    send_rplidar_command,
    SL_LIDAR_CMD_GET_DEVICE_INFO,
    SL_LIDAR_CMD_GET_DEVICE_HEALTH,
    SL_START_FLAG,
    SL_LIDAR_CMD_SCAN,
    SL_LIDAR_CMD_STOP
)

class LidarCommNode(Node):
    def __init__(self):
        super().__init__('lidar_comm_node')
        
        self.serial_helper = SerialCommNode()
        lidar_port = self.serial_helper.connected_ports.get('RP')
        
        if not lidar_port:
            self.get_logger().error(" Cannot find LiDAR")
            return
        
        try:
            # RPLIDAR 기본 Baudrate: 1,024,000
            self.ser = serial.Serial(lidar_port, baudrate=1024000, timeout=1)
            self.get_logger().info(f"✅ LIDAR 연결됨: {lidar_port}")

            # 1) Device Info 요청
            self.get_logger().info("📤 GET_DEVICE_INFO 전송")
            self.ser.write(bytes([SL_START_FLAG, SL_LIDAR_CMD_GET_DEVICE_INFO]))
            info_resp = self.ser.read(30)  # 응답 헤더+페이로드
            self.get_logger().info(
                "📨 Device Info RAW: " + ' '.join(f"{b:02X}" for b in info_resp)
            )

            # 2) Health 요청
            self.get_logger().info("📤 GET_HEALTH 전송")
            self.ser.write(bytes([SL_START_FLAG, SL_LIDAR_CMD_GET_DEVICE_HEALTH]))
            health_resp = self.ser.read(7)  # 최소 헤더 길이
            self.get_logger().info(
                "📨 Health RAW: " + ' '.join(f"{b:02X}" for b in health_resp)
            )

            # 3) Scan 명령 예시 (필요 시)
            self.get_logger().info("📤 SCAN 전송")
            self.ser.write(bytes([SL_START_FLAG, SL_LIDAR_CMD_SCAN]))
            scan_header = self.ser.read(7)
            self.get_logger().info(
                "📨 Scan Header RAW: " + ' '.join(f"{b:02X}" for b in scan_header)
            )
            
            # 4) Stop 명령 예시 (필요 시)
            self.get_logger().info("📤 STOP 전송")
            self.ser.write(bytes([SL_START_FLAG, SL_LIDAR_CMD_STOP]))
            scan_header = self.ser.read(7)
            self.get_logger().info(
                "📨 Scan Header RAW: " + ' '.join(f"{b:02X}" for b in scan_header)
            )

        except Exception as e:
            self.get_logger().error(f"❌ LIDAR 통신 실패: {e}")
            
def main(args=None):
    rclpy.init(args=args)
    node = LidarCommNode()
    rclpy.spin_once(node, timeout_sec=2.0)
    node.destroy_node()
    rclpy.shutdown()            