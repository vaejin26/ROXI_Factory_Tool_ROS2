import rclpy
import serial
import time
from rclpy.node import Node
from roxi_communication.ZLAC8015D import Controller
from roxi_communication.serial_comm_node import SerialCommNode

class MotorCommNode(Node):
    
    def __init__(self):
        super().__init__('motor_comm_node')
        
        self.serial_helper = SerialCommNode()
        motor_port = self.serial_helper.connected_ports.get('MOT')
        
        if not motor_port:
            self.get_logger().error("Cannot find MOT port")
            return
        
        try:
            self.ser = serial.Serial(motor_port, baudrate=115200, timeout=1)
            self.get_logger().info(f"MOT connected: {motor_port}")
        except Exception as e:
            self.get_logger().error(f"Cannot connect to MOT: {e}")
            return
        
        self.motors = Controller()
        self.last_time = time.time()
        self.Motorperiod = 0.0

        self.motor_timer = self.create_timer(0.05, self.on_motor_timer_timeout)
        self.on_Mot_state_changed()
        
        
    def on_motor_timer_timeout(self):
        if self.motors.client and self.motors.client.is_socket_open():
            rpmL, rpmR = self.motors.get_rpm()
            (L_fault_flag, L_fault_code), (R_fault_flag, R_fault_code) = self.motors.get_fault_code()
            if L_fault_flag == 0 and R_fault_flag == 0:
                self.Motorperiod = time.time() - self.last_time
                self.last_time = time.time()
                self.get_logger().info("period: {:.4f} rpmL: {:.1f} | rpmR: {:.1f}".format(self.Motorperiod, rpmL, rpmR))
            else:                
                error_info = ''
                if L_fault_flag:
                    error_info += " Left Motor: " + self.MotorErrorCode(L_fault_code)
                if R_fault_flag:
                    error_info += " Right Motor: " + self.MotorErrorCode(R_fault_code)
                self.get_logger().warn(error_info)
        else:
            self.get_logger().error("Motor is not connected")
            self.motors.SerialportClosed()


    def on_Mot_state_changed(self, state=None):
        if self.motors.client and self.motors.client.is_socket_open():
            self.motors.clear_alarm()
            self.motors.disable_motor()
            self.motors.set_accel_time(100, 100)
            self.motors.set_decel_time(100, 100)
            self.motors.set_mode(3)
            self.motors.enable_motor()
            self.motors.set_rpm(10, 10)
            self.get_logger().info("Motor initialized and running.")
        else:
            self.get_logger().error("Motor client is not connected or socket is closed.")

    def MotorErrorCode(self, Code):
        error_codes = {
            0x0000: "NO_FAULT",
            0x0001: "OVER_VOLT",
            0x0002: "UNDER_VOLT",
            0x0004: "OVER_CURR",
            0x0008: "OVER_LOAD",
            0x0010: "CURR_OUT_TOL",
            0x0020: "ENCOD_OUT_TOL",
            0x0040: "MOTOR_BAD",
            0x0080: "REF_VOLT_ERROR",
            0x0100: "EEPROM_ERROR",
            0x0200: "WALL_ERROR",
            0x0400: "HIGH_TEMP"
        }
        return error_codes.get(Code, "UNKNOWN_FAULT")
        
def main(args=None):
    rclpy.init(args=args)
    node = MotorCommNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
