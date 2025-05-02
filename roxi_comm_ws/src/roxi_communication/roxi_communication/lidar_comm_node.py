import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String
from roxi_communication.serial_comm_node import SerialCommNode
from roxi_communication.comm_protocol import Get_MCU_Version_Info, parse_mcu_version_response

class RPCommNode(Node):
    def __init__(self):
        super().__init__('lidar_comm_node')

        # SerialCommNode 객체 생성 → 연결된 포트 정보 사용
        self.serial_helper = SerialCommNode()
        lidar_port = self.serial_helper.connected_ports.get('RP')

        if not lidar_port:
            self.get_logger().error("❌ MCU 포트를 찾을 수 없습니다.")
            return

        try:
            self.ser = serial.Serial(lidar_port, baudrate=1024000, timeout=1)
            self.get_logger().info(f"✅ Lidar 연결됨: {lidar_port}")

            Get_MCU_Version_Info(self.ser)
            self.get_logger().info("📤 MCU 버전 요청 전송 완료")

            response = self.ser.read_until(expected=b'\x03')
            self.get_logger().info("RAW HEX: " + ' '.join(f"{b:02X}" for b in response))

            parse_mcu_version_response(response)

        except Exception as e:
            self.get_logger().error(f"❌ MCU 통신 실패: {e}")
            
def main(args=None):
    rclpy.init(args=args)
    node = RPCommNode()
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()