import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time

from roxi_communication.serial_comm_node import SerialCommNode
from roxi_communication.ZLAC8015D import ZLAC8015D

class MotorCommNode(Node):
    def __init__(self):
        super().__init__('motor_comm_node')
        
        self.serial_helper = SerialCommNode()
        motor_port = self.serial_helper.connected_ports.get('MOT')
        
        if not motor_port:
            self.get_logger().error("Cannot find Motor port.")
            return
        
        if not motor_port.startswith("/dev/"):
            motor_port = "/dev/" + motor_port
            
        try:
            self.motor = ZLAC8015D()
            self.motor.SetSerialport(motor_port)
            
            if not self.motor.client or not self.motor.client.is_socket_open():
                raise RuntimeError("Failed to open port.")
            
            self.get_logger().info(f"Motor Connected: {motor_port}")
            
            self.motor.clear_alarm()
            self.motor.disable_motor()
            self.motor.set_accel_time(100, 100)
            self.motor.set_decel_time(100, 100)
            self.motor.enable_motor()
            self.motor.set_mode(3)
            
            self.motor.set_rpm(10, 10)
            self.get_logger().info("RPM Setted.")
            
        except Exception as e:
            self.motor = None
            self.get_logger().error(f"Failed to connect to ZLAC8015D: {e}")
            return
        
        # 주기적으로 상태 확인
        timer_period = 0.5  # 초 단위
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.publisher = self.create_publisher(Float32MultiArray, 'motor_status', 10)

        
    def timer_callback(self):
        try:
            rpmL, rpmR = self.motor.get_rpm()
            (L_fault_flag, L_fault_code), (R_fault_flag, R_fault_code) = self.motor.get_fault_code()
            
            if L_fault_flag == 0 and R_fault_flag == 0:
                msg = Float32MultiArray()
                msg.data = [rpmL, rpmR]
                self.publisher.publish(msg)
                self.get_logger().info(f"📤 rpmL: {rpmL:.1f} | rpmR: {rpmR:.1f}")
            else:
                self.get_logger().warn(f"⚠️ 모터 오류 - L:{L_fault_code}, R:{R_fault_code}")

        except Exception as e:
            self.get_logger().error(f"❌ 모터 상태 확인 중 오류: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MotorCommNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'motor') and node.motor.client.is_socket_open():
            node.motor.set_rpm(0, 0)
            node.motor.SerialportClosed()
        node.destroy_node()
        rclpy.shutdown()