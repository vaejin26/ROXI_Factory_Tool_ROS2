import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String
from roxi_communication.serial_comm_node import SerialCommNode
from roxi_communication.comm_protocol import Get_MCU_Version_Info, parse_mcu_version_response

SL_START_FLAG                   = 0xA5
# // Commands        
SL_LIDAR_AUTOBAUD_MAGICBYTE      =   0x41
# // Commands without payload and response
SL_LIDAR_CMD_STOP               =    0x25
SL_LIDAR_CMD_SCAN               =    0x20
SL_LIDAR_CMD_FORCE_SCAN         =    0x21
SL_LIDAR_CMD_RESET              =    0x40
# // Commands with payload but no response
SL_LIDAR_CMD_NEW_BAUDRATE_CONFIRM  = 0x90 # added in fw 1.30
# // Commands without payload but have response
SL_LIDAR_CMD_GET_DEVICE_INFO      =  0x50
SL_LIDAR_CMD_GET_DEVICE_HEALTH    =  0x52
SL_LIDAR_CMD_GET_SAMPLERATE       =  0x59 #//added in fw 1.17
SL_LIDAR_CMD_HQ_MOTOR_SPEED_CTRL  =  0xA8

# // Commands with payload and have response
SL_LIDAR_CMD_EXPRESS_SCAN          = 0x82 #//added in fw 1.17
SL_LIDAR_CMD_HQ_SCAN               = 0x83 #//added in fw 1.24
SL_LIDAR_CMD_GET_LIDAR_CONF        = 0x84 #//added in fw 1.24
SL_LIDAR_CMD_SET_LIDAR_CONF        = 0x85 #//added in fw 1.24
# //add for A2 to set RPLIDAR motor pwm when using accessory board
SL_LIDAR_CMD_SET_MOTOR_PWM          = 0xF0
SL_LIDAR_CMD_GET_ACC_BOARD_FLAG     = 0xFF

IDLE_S = 1

u8RpLidarSendCmd = 0

class RPCommNode(Node):
    def __init__(self):
        super().__init__('lidar_comm_node')

        # SerialCommNode 객체 생성 → 연결된 포트 정보 사용
        self.serial_helper = SerialCommNode()
        lidar_port = self.serial_helper.connected_ports.get('RP')

        if not lidar_port:
            self.get_logger().error("❌ Lidar 포트를 찾을 수 없습니다.")
            return

        try:
            self.ser = serial.Serial(lidar_port, baudrate=1024000, timeout=1)
            self.get_logger().info(f"✅ Lidar 연결됨: {lidar_port}")



            response = self.ser.read_until(expected=b'\x03')
            self.get_logger().info("RAW HEX: " + ' '.join(f"{b:02X}" for b in response))

        except Exception as e:
            self.get_logger().error(f"❌ Lidar 통신 실패: {e}")
            
        self.lidar_timer = self.create_timer(0.05, self.on_RpLidar_timer_timeout)
    
    def on_RpLidar_timer_timeout(self):
        # This function is called every time the timer triggers
        self.lidar_timer.cancel()
        if self.ser and self.ser.is_open:
            self.lidar_timer.setInterval(50)         
            data = self.ser.readAll()  # Read available data            
            if data:   
                for i in range(0,data.size()): 
                    OneByte = int.from_bytes(bytes(data[i:i+1]), byteorder='big')                     
                    self.RpUartProcess(OneByte)
                    print(f"{OneByte:02x}",end =' ')
                print(f"<-- RpRx")

                
    def SendRplidarCommand(self,index): # Activate when controlling lidar
        '''
            SL_LIDAR_AUTOBAUD_MAGICBYTE      =   0x41
            # // Commands without payload and response
            SL_LIDAR_CMD_STOP               =    0x25
            SL_LIDAR_CMD_SCAN               =    0x20
            SL_LIDAR_CMD_FORCE_SCAN         =    0x21
            SL_LIDAR_CMD_RESET              =    0x40
            # // Commands with payload but no response
            SL_LIDAR_CMD_NEW_BAUDRATE_CONFIRM  = 0x90 # added in fw 1.30
            # // Commands without payload but have response
            SL_LIDAR_CMD_GET_DEVICE_INFO      =  0x50
            SL_LIDAR_CMD_GET_DEVICE_HEALTH    =  0x52
            SL_LIDAR_CMD_GET_SAMPLERATE       =  0x59 #//added in fw 1.17
            SL_LIDAR_CMD_HQ_MOTOR_SPEED_CTRL  =  0xA8

            # // Commands with payload and have response
            SL_LIDAR_CMD_EXPRESS_SCAN          = 0x82 #//added in fw 1.17
            SL_LIDAR_CMD_HQ_SCAN               = 0x83 #//added in fw 1.24
            SL_LIDAR_CMD_GET_LIDAR_CONF        = 0x84 #//added in fw 1.24
            SL_LIDAR_CMD_SET_LIDAR_CONF        = 0x85 #//added in fw 1.24
            # //add for A2 to set RPLIDAR motor pwm when using accessory board
            SL_LIDAR_CMD_SET_MOTOR_PWM          = 0xF0
            SL_LIDAR_CMD_GET_ACC_BOARD_FLAG     = 0xFF
        '''
        # if index == 0:
        #     print('SL_LIDAR_CMD_GET_DEVICE_HEALTH')
        #     self.RpLidarSendCmdWithOutPayload(SL_LIDAR_CMD_GET_DEVICE_HEALTH)
        # elif index == 1:
        #     print('SL_LIDAR_CMD_GET_DEVICE_INFO')
        #     self.RpLidarSendCmdWithOutPayload(SL_LIDAR_CMD_GET_DEVICE_INFO)
        # elif index == 2:
        #     print('SL_LIDAR_CMD_SCAN')
        #     self.RpLidarSendCmdWithOutPayload(SL_LIDAR_CMD_SCAN)
        # elif index == 3:
        #     print('SL_LIDAR_CMD_STOP')
        #     self.RpLidarSendCmdWithOutPayload(SL_LIDAR_CMD_STOP)
        print(index)
        if index == 0:
            print('SL_LIDAR_CMD_SCAN')
            self.RpLidarSendCmdWithOutPayload(SL_LIDAR_CMD_SCAN)
        elif index == 1:
            print('SL_LIDAR_CMD_STOP')
            self.RpLidarSendCmdWithOutPayload(SL_LIDAR_CMD_STOP)
        
            # self.ser  

    def SendCmd(self, index):
        try:
            print(index)
            self.SendRplidarCommand(index)       
        except ValueError as e:
            print(f"CMD select Error 0~2 : {e}")
            return 0
        
        return 0

    def RpLidarSendCmdWithOutPayload(self,Cmd):
        ucSendDat = bytearray(40)  # Data buffer          

        # Populate ucSendDat with data
        ucSendDat[0] = SL_START_FLAG   # start flag
        ucSendDat[1] = Cmd  # Length
        u8RpLidarSendCmd = Cmd
        ucSendDat[2] = 0x00    
        if self.ser and self.ser.isOpen():
            qbyte_send = ucSendDat[:3]
            self.ser.write(qbyte_send)  # Send data
            self.ser.flush()  
            print(f"{qbyte_send}", end=", ")
        else:
            self.get_logger().error("❌ RPlidarserial_port is not connected!")

        print(f'RpLidarCmd : {Cmd}')


    # Updating Lidar version label
    def parse_device_info(self,data):
        """장치 정보 데이터 파싱"""
        model = data[0]
        firmware_version = (data[2] << 8) | data[1]
        hardware_version = data[3]
        serial_number = ''.join(format(byte, '02X') for byte in data[4:20])

        return {
            "Model": f"{model:02x}",
            "Firmware Version": f"{firmware_version >> 8}.{firmware_version & 0xFF}",
            "Hardware Version": hardware_version,
            "Serial Number": serial_number
        }
            #Lidar processing logic
            
    def RpUartProcess(self, rxOneDat):
        global uartRpstate,uartRpCnt, cRpBuff
        PRLIDAR_PROTOCOL_REQUEST_TYPE = 2
        PRLIDAR_PROTOCOL_REQUEST_LEN  = 3
        PRLIDAR_PROTOCOL_REQUEST_DAT  = 4
        cRpBuff[uartRpCnt] = rxOneDat; 
        uartRpCnt += 1
        uartRpCnt &= 0xffff
        if uartRpstate == IDLE_S:
            if rxOneDat == SL_START_FLAG:
                uartRpstate            = PRLIDAR_PROTOCOL_REQUEST_TYPE
                uartRpCnt              = 1
                cRpBuff[0] = rxOneDat
            else:                
                uartRpCnt   = 0
                uartRpstate = IDLE_S
        elif uartRpstate == PRLIDAR_PROTOCOL_REQUEST_TYPE:
               uartRpstate = PRLIDAR_PROTOCOL_REQUEST_LEN
        elif uartRpstate == PRLIDAR_PROTOCOL_REQUEST_LEN:
            # if rxOneDat == PRLIDAR_PROTOCOL_REQUEST_LEN :
                uartRpstate            = PRLIDAR_PROTOCOL_REQUEST_DAT
        elif uartRpstate == PRLIDAR_PROTOCOL_REQUEST_DAT:
            # print("Protocol."+f"{uartRpCnt}"+f" {cRpBuff[2] + 6}")
            if uartRpCnt > cRpBuff[2] + 6:
                # print("valid PRLIDAR protocol."+f"{cRpBuff[1]:02x} "+f"{cRpBuff[2]}"+f"=={uartRpCnt}")
                if cRpBuff[1] == 0x5A and u8RpLidarSendCmd == SL_LIDAR_CMD_GET_DEVICE_INFO:
                    print(self.parse_device_info(cRpBuff[7:30])) 
                    # self.gui.terminal_output.append(f"{self.parse_device_info(cRpBuff[7:30])}") 
                    

                uartRpstate = IDLE_S
                uartRpCnt   = 0             
            
def main(args=None):
    rclpy.init(args=args)
    node = RPCommNode()
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()