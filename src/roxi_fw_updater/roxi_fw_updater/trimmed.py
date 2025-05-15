import rclpy
from rclpy.node import Node
import serial
import time
import os

verdateTime = '(2025.1.7.)'
mucversion = '0.1 '
# Define state constants
START_S = 0
IDLE_S = 1
LEN_S = 2
PIDX_S = 3
DATA_S = 4
CRC_S = 5
ETX_S = 6
SEND_S = 7

WAIT_Proc_SW_S = 8
WAIT_Proc_Door_S = 9
WAIT_Proc_US_S = 10
WAIT_Proc_Battery_S = 11
WAIT_Proc_Version_S = 12
WAIT_Proc_LED_ctrl_S = 13

RSV_S = 14

# Define packet index constants
P_STX = 0
P_LEN = 1
P_Idx = 2
P_CMD = 3
P_Dat = 4
P_CRC1 = 5
P_CRC2 = 6
P_ETX = 7
P_SEND = 8

u8CmdSetReadyToUpdateCtrl = 0x97

GetMcuVersionInformation  = 0xA4
GetTRAYSensorDATA         = 0xA6
u8CmdSetTrayBaseLedCtrl   = 0x96
SendReport                = 0xA3
u8CmdSetReportSensor      = 0xA3

# Initialize variables
uart5state = IDLE_S

# STX = 0x02  # Define the start byte
ETX = 0x03  # Define the end byte

SOH = b'\x01'
STX = b'\x02'
EOT = b'\x04'
ACK = b'\x06'
DLE = b'\x10'
NAK = b'\x15'
CAN = b'\x18'
CRC = b'C'

u8Uart_TOFDebugPrintflag = 2

u8SendPacketIndex  = 0

Xmodemstate = 0

FwBaseBin = None
FwTrayBin = None
FwDownBin = None

#---------------------------------------------------------------------------Until here

# Define the crc16 function in Python
def crc16(pData, length):
    wCrc = 0xffff
    for byte in pData[:length]:
        wCrc ^= byte << 8
        for _ in range(8):
            if wCrc & 0x8000:
                wCrc = (wCrc << 1) ^ 0x1021
            else:
                wCrc <<= 1
    return wCrc & 0xffff

class SerialReaderThread(QThread):
    # Protocol bytes

    data_received = pyqtSignal(bytes)   
    def __init__(self, serial_port):
        super().__init__()        
        self.serial_port = serial_port     
        self.uartRxWriteIdx = 0
        self.uartRxReadIdx = 0
        self.RxBuff = bytearray(256)

    def readData(self):
        if self.uartRxReadIdx != self.uartRxWriteIdx:
            OneByte = self.RxBuff[self.uartRxWriteIdx]
            self.uartRxWriteIdx += 1
            self.uartRxWriteIdx &= 0xff
            return OneByte
        else:            
            return 'NAN'
    def run(self):
        while self.serial_port.isOpen():
            #if self.serial_port.waitForReadyRead(100):  # Wait for data with a timeout of 100ms
            self.serial_port.waitForReadyRead(10)
            data = self.serial_port.readAll()  # Read available data            
            if data:                                                
                #int_data = list(map(lambda byte: byte, data))  # Each byte is already an integer
                for i in range(0,data.size()): 
                    OneByte = int.from_bytes(bytes(data[i:i+1]), byteorder='big')           
                    self.RxBuff[self.uartRxWriteIdx] = OneByte
                    self.uartRxWriteIdx += 1
                    self.uartRxWriteIdx &= 0xff
                #     if Xmodemstate != 0 :
                #         self.handle_xmodem(OneByte)
                #     else:
                #         uart_process((OneByte))                    


class SerialPortApp(QWidget):
    def __init__(self):
        super().__init__()
        self.serial_port            = None

        global FwBaseBin,FwDownBin,FwTrayBin

        self.is_reconnecting = False        

        # Set up a QTimer
        self.timer = QTimer()
        self.timer.setInterval(100)  # Set timer to trigger every 1000ms (1 second)
        self.timer.timeout.connect(self.on_timer_timeout)  # Connect timeout to the handler function

        self.XmodemValueInitialize()

        self.FindSerialPortInfo()
        
        setCurrentIndex = 0

        # Get System version 
        if self.serial_port and self.serial_port.isOpen():                                           
                self.GetMCU_VERSION_INFO()
                print('Get MCU version') 

    def FindSerialPortInfo(self):
        if(self.update_serial_ports("ttyUSB1",self.comboMCU) == False):
            self.ConnectLabel[0].setStyleSheet("background-color: red; color: black;")
            self.ConnectLabel[0].setText('MCU :Error')
        else:
            self.ConnectLabel[0].setStyleSheet("background-color: green; color: black;")
            self.ConnectLabel[0].setText('MCU :Found')
            self.toggle_connection()

    def US_And_SW_Battey_info(self,Ypos,Xpos):
        YposT = Ypos
        XposT = Xpos
        print(YposT)
        for i in range(8):        
            self.USLabel[i].move(XposT+82 * i , YposT)            
        for i in range(3):        
            self.SWLabel[i].move(XposT+82 * i , YposT+16)
        self.BatteryLabel.move(XposT+82*3 , YposT+16)
        return YposT +60
    
    def uart_process(self,cc):
        global uart5state, uartTrayCnt, cTofBuff,Xmodemstate
        
        print(f"{cc:02x}",end =' ')
        if(uartTrayCnt < 254) :
            uartTrayCnt =  uartTrayCnt + 1
        
        cTofBuff[uartTrayCnt] = cc; 
        if uart5state == IDLE_S:
            # Check for start byte (STX)
            if cc == 0x2:
                uart5state = LEN_S
                uartTrayCnt = 0
                cTofBuff[uartTrayCnt] = cc

        elif uart5state == LEN_S:
            # Validate length of packet
            if cc > 249 or cc < 1:
                uart5state = IDLE_S  # Reset state if length is invalid
                uartTrayCnt = 0
            else:
                uart5state = PIDX_S

        elif uart5state == PIDX_S:
            uart5state = DATA_S  # Move to data state

        elif uart5state == DATA_S:
            if uartTrayCnt >= (cTofBuff[P_LEN] + 2):
                uart5state = CRC_S  # Move to CRC state

        elif uart5state == CRC_S:
            if uartTrayCnt >= (cTofBuff[P_LEN] + 4):
                uart5state = ETX_S  # Move to ETX state

        elif uart5state == ETX_S:
            uart5state = IDLE_S  # Reset state to IDLE                
            if cc == ETX:
                # Calculate CRC and verify it
                crc_dat1 = crc16(cTofBuff , uartTrayCnt - 2)
                crc_dat2 = (cTofBuff[uartTrayCnt - 1] << 8) | cTofBuff[uartTrayCnt - 2]
                if crc_dat1 == crc_dat2:
                    uart5state = IDLE_S # CRC matches, move to reserved state
                    if(u8CmdSetReadyToUpdateCtrl == cTofBuff[3]):
                        Xmodemstate = 1                    
                        print("Ready to update")                    
                    elif(GetMcuVersionInformation == cTofBuff[3]):
                        startPos = P_Dat
                        ver_year  = [0,0,0,0,0]
                        ver_month = [0,0,0,0,0]
                        ver_date  = [0,0,0,0,0]
                        ver_major = [0,0,0,0,0]
                        ver_minor = [0,0,0,0,0]
                        print("GetMcuVersionInformation")
                        print(cTofBuff)
                        for i in range(5):
                            ver_year[i]  = (cTofBuff[startPos + 1] << 8) | (cTofBuff[startPos + 0])
                            ver_month[i] = (cTofBuff[startPos + 2])
                            ver_date[i] = (cTofBuff[startPos + 3])
                            ver_major[i]  = (cTofBuff[startPos + 5] << 8) | (cTofBuff[startPos + 4])
                            ver_minor[i]  = (cTofBuff[startPos + 7] << 8) | (cTofBuff[startPos + 6])
                            startPos = startPos + 8                              
                else:
                    if u8Uart_TOFDebugPrintflag == 2:
                        print(f"TOF CRC Error --> {cc:02x} : {crc_dat1:04x} {crc_dat2:04x}")

    def handle_error(self, error):
        """Handle serial port error (e.g., connection lost)"""
        if error == QSerialPort.ResourceError:            
            self.textEdit.append(f"Status: Connection lost!")
            print("Serial port connection lost!")
            self.toggle_connection()
            self.is_reconnecting = True

    def XmodemValueInitialize(self):
        global Xmodemstate
        Xmodemstate = 0
        self.error_count = 0
        self.quiet=False
        self.cancel = 0
        self.crc_mode = -1
        self.error_count = 0
        self.success_count = 0
        self.total_packets = 0
        self.sequence = 1
        self.packet_size = 128
        self.RecivedData = 0

    def abort(self, count=2, timeout=60):
        for _ in range(count):
            self.putc(CAN, timeout)
    
    def _make_send_header(self, packet_size, sequence):
        assert packet_size in (128, 1024), packet_size
        _bytes = []
        if packet_size == 128:
            _bytes.append(ord(SOH))
        elif packet_size == 1024:
            _bytes.append(ord(STX))
        _bytes.extend([sequence, 0xff - sequence])
        return bytearray(_bytes)

    def x_calc_check(self,pkt_buf):
        check = 0
        for i in range(128):
            check = self.update_crc(pkt_buf[i], check)
        return check

    def update_crc(self,b, crc):
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
        return crc & 0xFFFF  # Ensure CRC remains a 16-bit value

    def _make_send_checksum(self, crc_mode, data):
        _bytes = []
        if crc_mode:
            crc = self.x_calc_check(data)# self.calc_crc(data)
            _bytes.extend([crc >> 8, crc & 0xff])
            # print("if",_bytes)
        else:
            crc = self.calc_checksum(data)
            _bytes.append(crc)            
        
        return bytearray(_bytes)    

    def calc_checksum(self, data, checksum=0):
        if platform.python_version_tuple() >= ('3', '0', '0'):
            return (sum(data) + checksum) % 256
        else:
            return (sum(map(ord, data)) + checksum) % 256

    def calc_crc(self, data, crc=0):
        for char in bytearray(data):
            crctbl_idx = ((crc >> 8) ^ char) & 0xff
            crc = ((crc << 8) ^ self.crctable[crctbl_idx]) & 0xffff
        return crc & 0xffff
    
    def handle_xmodem(self , cc):        
        global FwBaseBin,FwTrayBin,Xmodemstate
        retry=16
        char = cc.to_bytes(1, byteorder='big')
        print(f"char = {char}")
        # print(char,Xmodemstate,self.total_packets)            
        if Xmodemstate == 1:
            if char:
                if char == NAK:
                    print('standard checksum requested (NAK).')
                    self.crc_mode = 0
                    Xmodemstate = 2
                    return True
                elif char == CRC:
                    print('16-bit CRC requested (CRC).')
                    self.crc_mode = 1
                    Xmodemstate = 2
                    self.progress_bar.setVisible(True) 
                    self.progress_bar.setValue(0)
                    self.sendButton.setEnabled(False)
                    return True
                elif char == CAN:
                    if not self.quiet:
                        print('received CAN', file=sys.stderr)
                    if self.cancel:
                        print('Transmission canceled: received CAN CAN '
                                        'at start-sequence')
                        return False
                    else:
                        print('received CAN at start of sequence.')
                        self.cancel = 1
                        Xmodemstate = 0
                elif char == EOT:
                    print('Transmission canceled: received EOT ''at start-sequence')
                    return False
                else:
                    print('send error: expected NAK, CRC, EOT or CAN; '
                                    'got %r', char)

            self.error_count += 1
            if self.error_count > retry:
                print('send error: error_count reached %d, aborting.', retry)
                self.abort(timeout=60)
                Xmodemstate = 0
                self.error_count = 0
                return False
        elif Xmodemstate == 3:
            if char == ACK:
                self.total_packets += 1
                self.success_count += 1
                self.error_count = 0
                # keep track of sequence
                self.sequence = (self.sequence + 1) % 0x100
                Xmodemstate = 2
            else:
                self.error_count += 1
                Xmodemstate = 3
                if self.error_count > retry:
                    Xmodemstate = 0
        elif Xmodemstate == 4:
            if char == ACK:
                self.error_count = 0
                Xmodemstate = 0
                print('send EOT Get ACK; ') 
            else :
                print('send error: expected ACK; got %r', char)                
                self.serial_port.write(EOT)
                self.error_count += 1
                if self.error_count > retry:
                    Xmodemstate = 0
                    print('send EOT Get ACK; ') 

        if Xmodemstate == 2:  
            global FwDownBin          
            data = bytearray(128) 
            endofData = False
            self.progress_bar.setValue(int(self.total_packets*self.packet_size/len(FwDownBin) * 100))
            # print(len(FwDownBin))
            for i in range(self.packet_size):
                if(len(FwDownBin) > self.total_packets*self.packet_size + i):
                    data[i] = FwDownBin[self.total_packets*self.packet_size + i]
                    # print(f"{FwDownBin[i]:02x}", end=", ")                    
                else :
                    data[i] = 0x1A
                    endofData = True
            if endofData == False:
                # print('self.total_packets',self.total_packets)
                header = self._make_send_header(self.packet_size, self.sequence)
                checksum = self._make_send_checksum(self.crc_mode, data)
                sendData = (header + data + checksum)
                self.serial_port.write(sendData)                      
                # for i in bytearray(sendData):
                #     print(f"{i:02x}", end=", ")   
                # self.serial_port.waitForReadyRead(10)         
                # indata = self.serial_port.readAll()  # Read available data                                     
                Xmodemstate = 3                                
            else :
                self.progress_bar.setVisible(False) 
                Xmodemstate = 0
                self.XmodemValueInitialize()
                self.sendButton.setEnabled(True)
                self.serial_port.write(EOT)
                # end of stream
                print('send: at EOF')
                
    def parse_device_info(self,data):
        """장치 정보 데이터 파싱"""
        model = data[0]
        firmware_version = (data[2] << 8) | data[1]
        hardware_version = data[3]
        serial_number = ''.join(format(byte, '02X') for byte in data[4:20])

        self.ChangeLableColor(self.VerLabel[2],firmware_version)
        self.VerLabel[2].setText(f"RPLidar:FW"+f"{firmware_version >> 8}.{firmware_version & 0xFF}"+",SN: "+serial_number)
        return {
            "Model": f"{model:02x}",
            "Firmware Version": f"{firmware_version >> 8}.{firmware_version & 0xFF}",
            "Hardware Version": hardware_version,
            "Serial Number": serial_number
        }
    
    def on_timer_timeout(self):
        global Xmodemstate
        # This function is called every time the timer triggers
        self.timer.stop()
        # self.label.setText("Timer triggered!")  # Update the label or perform any action
        if self.serial_port.isOpen():
            self.timer.setInterval(100)
            self.is_reconnecting = False
            #if self.serial_port.waitForReadyRead(100):  # Wait for data with a timeout of 100ms            
            data = self.serial_port.readAll()  # Read available data
            if data:    
                for i in range(0,data.size()): 
                    OneByte = int.from_bytes(bytes(data[i:i+1]), byteorder='big') 
                    if Xmodemstate != 0 :
                        
                        self.handle_xmodem(OneByte)
                        
                    else:
                        self.uart_process(OneByte)
            if(self.connectButton.text() == 'MCU_Disconnect'):
                self.timer.start()         

    def update_serial_ports(self, findPortName, ComboBox):
        ports = QSerialPortInfo.availablePorts()
        ComboBox.clear()
        index = -1

        for idx, port in enumerate(ports):
            description = port.description()
            port_display = f"{port.portName()} ({description})" if description else port.portName()

            if findPortName in port.portName() or findPortName in port.serialNumber():
                index = idx

        if index != -1:
            ComboBox.setCurrentIndex(index)
            return True
        return False
    
    def on_port_selected(self):
        # Called when a new port is selected from the combo box
        if self.serial_port and self.serial_port.isOpen():
            self.serial_port.close()  # Close the currently opened port if any
    
    def send_serial_data(self):
        global FwDownBin
        self.SendCmd(self)

    def openFileDialog(self):
        global FwBaseBin,FwTrayBin
        # Open a file dialog to select a binary file
        file_name, _ = QFileDialog.getOpenFileName(self, 'Open File', '', 'Binary Files (*.bin);;All Files (*)')
        
        if file_name:
            if self.sender() == self.openButton1:
                print(file_name)
                with open(file_name, 'rb') as file:
                    FwBaseBin = file.read()                    
                self.fileLabel1.setText(f"File 1: {os.path.basename(file_name)}")  # Show selected file 1 name
                print("Base File Size",len(FwBaseBin))
            elif self.sender() == self.openButton2:
                print(file_name)
                with open(file_name, 'rb') as file:
                    FwTrayBin = file.read()                                     
                self.fileLabel2.setText(f"File 2: {os.path.basename(file_name)}")  # Show selected file 2 name
                print("Tray File Size",len(FwTrayBin))


    def GetMCU_VERSION_INFO(self):       
        global u8SendPacketIndex  # Access the global index variable
        ucSendDat = bytearray(40)  # Data buffer                           
        # except ValueError as e:
        #     print(f"Invalid input: {e}")
        #     return 0
        
        # Populate ucSendDat with data
        ucSendDat[P_STX] = 0X02  # Start byte
        ucSendDat[P_LEN] = 1   # Length
        ucSendDat[P_Idx] = u8SendPacketIndex
        u8SendPacketIndex += 1
        u8SendPacketIndex &= 0xFF
        ucSendDat[P_CMD] = GetMcuVersionInformation        
        # Calculate CRC for the data segment
        Datidx = P_CMD + 1
        crc = crc16(ucSendDat[3:Datidx], ucSendDat[P_LEN])
        ucSendDat[Datidx] = crc & 0xFF  # CRC low byte
        ucSendDat[Datidx + 1] = (crc >> 8) & 0xFF  # CRC high byte
        ucSendDat[Datidx + 2] = ETX  # End byte        
        # Print each byte for debugging
        for i in range(Datidx + 3):
            print(f"{ucSendDat[i]:02x}", end=", ")
        print("\n GetMCU_VERSION_INFO")
        if self.serial_port and self.serial_port.isOpen():
            qbyte_send = ucSendDat[:Datidx+3]
            self.serial_port.write(qbyte_send)  # Send data
        else:
            self.textEdit.append("Serial port is not connected!")



    def SendUpdateCmd(self):
        global FwBaseBin,FwTrayBin,FwDownBin
        global u8SendPacketIndex  # Access the global index variable
        ucSendDat = bytearray(40)  # Data buffer        
        # Get values from QLineEdit widgets, converting them to integers
        try:
        # Ensure values are within the acceptable range            
            if (self.combo_box.currentIndex() == 0 ):   
                    if(len(FwBaseBin) == 0):
                        raise ValueError("Base FW File not Read")
                    else:
                        FwDownBin = FwBaseBin
                        print(f"Start Down load Base.Bin")
            elif (self.combo_box.currentIndex() < 5):                
                    if(len(FwTrayBin) == 0):
                        raise ValueError("Base FW File not Read")
                    else:                        
                        FwDownBin = FwTrayBin
                        print(f"Start Down load Tray.Bin")
            else:
                raise ValueError("Net Setting for fw mode(0~4)")
        except ValueError as e:
            print(f"Send file is empty : {e}")
            return 0
        # Populate ucSendDat with data
        ucSendDat[P_STX] = 0X02  # Start byte
        ucSendDat[P_LEN] = 3   # Length
        ucSendDat[P_Idx] = u8SendPacketIndex
        u8SendPacketIndex += 1
        u8SendPacketIndex &= 0xFF
        ucSendDat[P_CMD] = u8CmdSetReadyToUpdateCtrl
        ucSendDat[P_Dat + 0] = self.combo_box.currentIndex()
        ucSendDat[P_Dat + 1] = 1 # set Mcu Update
        print(f"here: {u8SendPacketIndex}")
        print(f"here: {P_Dat}")

        # Calculate CRC for the data segment
        Datidx = P_Dat + 2
        crc = crc16(ucSendDat[3:Datidx], ucSendDat[P_LEN])
        ucSendDat[Datidx] = crc & 0xFF  # CRC low byte
        ucSendDat[Datidx + 1] = (crc >> 8) & 0xFF  # CRC high byte
        ucSendDat[Datidx + 2] = ETX  # End byte
        
        # Print each byte for debugging
        for i in range(Datidx + 3):
            print(f"{ucSendDat[i]:02x}", end=", ")
        print("\n u8CmdSetReadyToUpdateCtrl")
        if self.serial_port and self.serial_port.isOpen():
            qbyte_send = ucSendDat[:Datidx+3]
            self.serial_port.write(qbyte_send)  # Send data

    def SendCmd(self):        
        try:
            print(self.combo_box_Cmd.currentIndex())
            self.SendRplidarCommand(self.combo_box_Cmd.currentIndex())       
        except ValueError as e:
            print(f"CMD select Error 0~2 : {e}")
            return 0
        
        return 0

if __name__ == '__main__':
    # Create the application instance
    app = QApplication(sys.argv)

    # Create and show the main window
    window = SerialPortApp()
    window.show()

    # Start the application event loop
    sys.exit(app.exec_())

