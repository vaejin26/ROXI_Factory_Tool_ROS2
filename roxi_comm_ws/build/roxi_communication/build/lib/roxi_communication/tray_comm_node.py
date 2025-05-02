import rclpy
from rclpy.node import Node
import serial
from roxi_communication.serial_comm_node import SerialCommNode
from roxi_communication.comm_protocol import Get_Tray_Sensor_Data

class TrayCommNode(Node):
    def __init__(self):
        super().__init__('tray_comm_node')
        
        self.serial_helper = SerialCommNode()
        tray_port = self.serial_helper.connected_ports.get('TRAY')
        
        if not tray_port:
            self.get_logger().error("❌ Tray 포트를 찾을 수 없습니다.")
            return

        try:
            self.ser = serial.Serial(tray_port, baudrate=115200, timeout=1)
            self.get_logger().info(f"✅ Tray 연결됨: {tray_port}")

            Get_Tray_Sensor_Data(self.ser)
            self.get_logger().info("📤 Tray 센서 데이터 요청 전송 완료")

            # 아직 응답 파싱은 불가 → raw read만 진행
            response = self.ser.read(32)
            self.get_logger().info("RAW (예상 응답): " + ' '.join(f"{b:02X}" for b in response))

        except Exception as e:
            self.get_logger().error(f"❌ Tray 통신 실패: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TrayCommNode()
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()