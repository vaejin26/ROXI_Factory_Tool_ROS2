import os
import sys
import platform
import re
import time
import binascii
import numpy as np
# The main function
from pathlib import Path
from PyQt5 import QtGui,QtCore
from PyQt5.QtCore import Qt, QThread, pyqtSignal,QTimer,QObject,QSize
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QFileDialog, QComboBox, QTextEdit, QLabel,QProgressBar,QGridLayout,QMainWindow
from PyQt5.QtWidgets import QCheckBox,QRadioButton,QButtonGroup,QTableWidgetItem,QTableWidget,QDialog,QDialogButtonBox
from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
from ctypes import c_int16
import ZLAC8015D
from PyQt5.QtGui import QPixmap


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

#define u8CmdSetLedCtrl        0x90
#define u8CmdSetMotorPower     0x91
#define u8CmdSetDoorCtrl       0x92
#define u8CmdSetChargingCtrl   0x93
#define u8CmdSetReportSensor   0x94
#define u8CmdSetAudioMuteCtrl       0x95
#define u8CmdSetTrayBaseLedCtrl     0x96
#define u8CmdSetReadyToUpdateCtrl   0x97

#define GetSwitchInfo               0xA0
#define GetUltraSonicSensorInfo     0xA1
#define GetBatteryVoltage           0xA2
#define SendReport                  0xA3
#define GetMcuVersionInformation    0xA4
#define GetLedCtrl                  0xA5
#define GetTRAYSensorDATA           0xA6

#define WIT_ACC         0x51
#define WIT_GYRO        0x52
#define WIT_ANGLE       0x53
#define WIT_MAGNETIC    0x54
#define WIT_DPORT       0x55
#define WIT_PRESS       0x56
#define WIT_GPS         0x57
#define WIT_VELOCITY    0x58
#define WIT_QUATER      0x59

u8CmdSetReadyToUpdateCtrl = 0x97

GetMcuVersionInformation  = 0xA4
GetTRAYSensorDATA         = 0xA6
u8CmdSetTrayBaseLedCtrl   = 0x96
SendReport                = 0xA3
u8CmdSetReportSensor      = 0xA3


WT61P_PROTOCOL_TYPE     = 2
WT61P_PROTOCOL_DATA     = 3
TypeAcceleration        = 0x51
TypeAngularvelocity     = 0x52
TypeAngle               = 0x53

# Initialize variables
uart5state = IDLE_S
uartTrayCnt = 0
cTofBuff = bytearray(256)  # Adjust the buffer size if needed
Tray_TofSensor = [[0 for _ in range(16)]for _ in range(4)]
Tray_UsSensor = [0 for _ in range(4)]
Tray_TofTotalSensor = [[0 for _ in range(17)]for _ in range(4)]

uartImustate = IDLE_S
uartImuCnt = 0
cImuBuff = bytearray(256)  # Adjust the buffer size if needed
 # Adjust the buffer size if needed


uartRpstate = IDLE_S
uartRpCnt   = 0
cRpBuff     = bytearray(65536) #



u8RpLidarSendCmd = 0
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

class ImagePopup(QDialog):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Need IMU Setup")
        self.setModal(True)
        self.resize(800, 600)

        # 레이아웃 설정
        layout = QVBoxLayout()

        # 같은 폴더의 이미지를 읽기        
        image_path = self.get_real_path("IMU_setupMenu.bmp")
        print("Image path:"+image_path)

        
        # 이미지 표시를 위한 QLabel
        self.image_label = QLabel(self)
        self.image_label.setScaledContents(True)
        self.image_label.setFixedSize(800, 600)

        # 이미지 로드 및 표시
        if os.path.exists(image_path):
            pixmap = QPixmap(image_path)
            self.image_label.setPixmap(pixmap.scaled(self.image_label.size(), aspectRatioMode=1))
        else:
            self.image_label.setText("IMU_setupMenu.bmp 파일을 찾을 수 없습니다.")

        layout.addWidget(self.image_label)

        # 닫기 버튼
        close_button = QDialogButtonBox(QDialogButtonBox.Close)
        close_button.rejected.connect(self.reject)
        layout.addWidget(close_button)
    def get_real_path(self,filename):
        if getattr(sys, 'frozen', False):  # PyInstaller로 패키징된 경우
            base_path = sys._MEIPASS
        else:  # 개발 환경에서 실행되는 경우
            base_path = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(base_path , filename)

def find_and_read_files(pattern, directory="."):
    """
    지정된 패턴과 일치하는 파일을 찾고 읽는 함수.
    
    :param pattern: 파일 이름 패턴 (예: 'Base_*.bin')
    :param directory: 탐색할 디렉토리 (기본값은 현재 디렉토리)
    """
    directory_path = Path(directory)

    try:
        for file_path in directory_path.glob(pattern):
            if file_path.is_file():  # 파일인지 확인
                print(f"Found file: {file_path}")
                return file_path 
               
    except Exception as e:
        print(f"Error: {e}")
    return None
   
class SerialReaderThread(QThread):
    # Protocol bytes

    data_received = pyqtSignal(bytes)   
    def __init__(self, serial_port):
        super().__init__()        
        self.serial_port = serial_port     
        self.uartRxWriteIdx = 0
        self.uartRxReadIdx = 0
        self.RxBuff = bytearray(256)
#         uartImuCnt = 0
# cIMUBuff = bytearray(256)  # Adjust the buffer size if needed
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

def Set3dplotData(SerialPortApp):
    # global Tray_TofSensor,Tray_UsSensor
    # SerialPortApp.update_all_plots(SerialPortApp,Tray_TofSensor)
    pass

# Signal class to communicate updates
class ArraySignal(QObject):
    updated = pyqtSignal(list)  # Signal to pass a list of NumPy arrays

# Define the 3D Bar Chart Canvas
class Bar3DCanvas(FigureCanvas):
    def __init__(self, parent=None):
        # Create a Matplotlib figure
        fig = Figure()
        self.ax = fig.add_subplot(111, projection='3d')
        super().__init__(fig)
        self.setParent(parent)

    def update_plot(self, data):
        self.ax.clear()  # Clear the previous plot
        x, y = np.meshgrid(np.arange(data.shape[0]), np.arange(data.shape[1]))
        x = x.flatten()  # Flatten to work with bar3d
        y = y.flatten()
        z = np.zeros_like(x)  # Base height is 0
        dx = dy = 0.5  # Width of bars
        dz = data.flatten()  # Heights of the bars

        # Plot the 3D bar chart
        self.ax.bar3d(x, y, z, dx, dy, dz, shade=True, cmap='viridis')
        self.ax.set_zlim(0, 2000)  # 최대값 설정 (z-axis 범위 제한)
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        # self.ax.set_zlabel("Value")
        self.draw()  # Redraw the canvas

class BarGraphCanvas(FigureCanvas):
    def __init__(self, parent=None):
        self.fig, self.ax = plt.subplots(figsize=(8, 4))
        super().__init__(self.fig)
        self.setParent(parent)

    def plot_bar_graph(self, data):
        """
        데이터를 사용해 바 그래프 그리기
        :param data: 1D 배열 형태의 데이터
        """
        self.ax.clear()  # 기존 그래프 초기화
        x = np.arange(len(data))  # X 좌표 (0, 1, ..., len(data)-1)
        bars = self.ax.bar(x, data, color='skyblue')  # 바 그래프 생성
        self.ax.set_ylim(0, 2000)  # 최대값 설정 (z-axis 범위 제한)

        for bar, value in zip(bars, data):
            self.ax.text(
                bar.get_x() + bar.get_width() / 2,  # 막대의 중앙 위치
                bar.get_height() + 0.5,             # 막대의 높이 위에
                str(value),                         # 값 표시
                ha='center', va='bottom', fontsize=9
            )
        # 라벨 및 제목 설정
        # self.ax.set_xlabel("Index", fontsize=10)
        # self.ax.set_ylabel("Value", fontsize=10)
        self.ax.set_title("Tray Sensor Info", fontsize=12)
        labels = ['US', '0', '1', '2', '3','4', '5', '6', '7','8', '9', '10', '11','12', '13', '14', '15' ]
        # x축 눈금 설정
        self.ax.set_xticks(x)
        self.ax.set_xticklabels(labels, rotation=90, fontsize=9)  # 사용자 정의 라벨

        self.draw()  # 그래프 업데이트

class SerialPortApp(QWidget):
    def __init__(self):
        super().__init__()
        self.serial_port            = None
        self.Trayserial_port        = None
        self.Imuserial_port         = None
        self.Motorserial_port       = None
        self.RPlidarserial_port     = None
        self.ImuRXthread = None
        self.MotRXthread = None
        self.RPlidarRXthread = None
        self.TrayRXthread = None
        
        self.u8Ledflag = 0
        self.u8LedTrayflag = 0
        self.ImuDataRxTimer = 0
        self.DP_POS_Y = 0
        self.initUI()
    def initUI(self):
        global FwBaseBin,FwDownBin,FwTrayBin
        # Create main layout
        main_layout = QVBoxLayout()

        
        # File selection layout
        # file_layout = QVBoxLayout()
        #self.load_serial_ports()
        # Text area for displaying received serial data
        # Serial_flag_layout = QVBoxLayout()
        # ComboBox for Serial Port selection

        self.comboMCU = QComboBox(self)
        self.comboMCU.setFixedSize(280, 30)

        self.comboTRAY = QComboBox(self)
        self.comboTRAY.setFixedSize(280, 30)

        self.comboIMU = QComboBox(self)
        self.comboIMU.setFixedSize(280, 30)

        self.comboMOT = QComboBox(self)
        self.comboMOT.setFixedSize(280, 30)

        self.comboRPLIDAR = QComboBox(self)
        self.comboRPLIDAR.setFixedSize(280, 30)

        
        # Button for serial port connection
        self.connectButton = QPushButton('MCU Connect', self)
        self.connectButton.clicked.connect(self.toggle_connection)
        self.connectButton.setFixedSize(150, 30)

        self.IMUconnectButton = QPushButton('IMU Connect', self)
        self.IMUconnectButton.clicked.connect(self.IMUtoggle_connection)
        self.IMUconnectButton.setFixedSize(150, 30)

        self.MOTconnectButton = QPushButton('MOT Connect', self)
        self.MOTconnectButton.clicked.connect(self.MOTtoggle_connection)
        self.MOTconnectButton.setFixedSize(150, 30)

        self.RPconnectButton = QPushButton('RP Connect', self)
        self.RPconnectButton.clicked.connect(self.RPtoggle_connection)
        self.RPconnectButton.setFixedSize(150, 30)

        

        # Add the progress bar to the layout
        # self.progress_bar.setValue()
        
        self.fileLabel1 = QLabel('No file selected', self)        
        self.fileLabel2 = QLabel('No file selected', self)
        self.openButton1 = QPushButton('Select Base(Binary)', self)
        self.openButton1.clicked.connect(self.openFileDialog)
        self.openButton2 = QPushButton('Select Tray(Binary)', self)
        self.openButton2.clicked.connect(self.openFileDialog)
        
        self.fileLabel1.setFixedSize(180,41)        
        self.openButton1.setFixedSize(120,41)        
      # 선택된 항목을 표시할 라벨
        self.label5 = QLabel("Select Download FW", self)
        self.label5.setFixedSize(130, 30)                
        # Button to send data
        self.sendButton = QPushButton('Start Update', self)
        self.sendButton.clicked.connect(self.SendUpdateCmd)
        self.sendButton.setFixedSize(100, 80)
        self.sendButton.setEnabled(True)
        self.fileLabel2.setFixedSize(180,41)        
        self.openButton2.setFixedSize(120,41)
        
       # 콤보박스 생성
        self.combo_box = QComboBox(self)
        self.combo_box.addItems(['Base', 'Tray1', 'Tray2', 'Tray3', 'Tray4'])
        self.combo_box.setFixedSize(130,30)        
        self.combo_box.setCurrentIndex(0)

        self.progress_bar = QProgressBar(self)
        self.progress_bar.setMinimum(0)
        self.progress_bar.setMaximum(100)
        self.progress_bar.setFixedSize(550, 15)
        self.progress_bar.setVisible(False) 

        Ypos = 10
        Xpos = 10
        self.connectButton.move(Xpos,Ypos) 
        Xpos += 151
        self.IMUconnectButton.move(Xpos,Ypos) 
        Xpos += 151
        self.MOTconnectButton.move(Xpos,Ypos) 
        Xpos += 151
        self.RPconnectButton.move(Xpos,Ypos) 

        Ypos = 10
        Xpos = 900
        self.comboMCU.move(Xpos,Ypos)                       
        Xpos += 280
        self.comboTRAY.move(Xpos,Ypos)        
        Xpos += 280
        self.comboIMU.move(Xpos,Ypos)
        Xpos += 280
        self.comboMOT.move(Xpos,Ypos)
        Xpos += 280
        self.comboRPLIDAR.move(Xpos,Ypos)

        

        self.AccelerationX = 0.0
        self.AccelerationY = 0.0
        self.AccelerationZ = 0.0
        self.Temperature   = 0.0

        self.AngularvelocityX   = 0.0
        self.AngularvelocityY   = 0.0
        self.AngularvelocityZ   = 0.0
        self.Voltage            = 0.0

        self.Roll   = 0.0
        self.Pitch  = 0.0
        self.Yaw    = 0.0
        self.WT61P_ver   = 0

        self.table_widget = []

        for i in range(4):
            table = QTableWidget(4, 2, self)  # 4x2 테이블
            table.setHorizontalHeaderLabels(["",""])  # 첫 번째 열 제목만 설정
            table.horizontalHeader().setDefaultAlignment(Qt.AlignCenter)
            table.verticalHeader().setVisible(False)  # 행 번호 숨기기
            table.setColumnWidth(0, 100)  # 첫 번째 열의 너비를 100으로 설정
            table.setColumnWidth(1, 150)  # 두 번째 열의 너비를 150으로 설정
            table.setFixedSize(QSize(255, 170))  # 고정 크기 설정
            table.move(10, 10 + (i * 180))  # 각 테이블 위치 이동 (위치 변경)
            self.table_widget.append(table)  # 리스트에 추가               

        # # end \
        item = QTableWidgetItem(f"Temp:")
        self.table_widget[0].setItem(3, 0, item)

        item = QTableWidgetItem(f"Voltage:")
        self.table_widget[1].setItem(3, 0, item)

        item = QTableWidgetItem(f"WT61P_ver:")
        self.table_widget[2].setItem(3, 0, item)

        item = QTableWidgetItem(f"Temp:")
        self.table_widget[3].setItem(3, 0, item)

        self.table_widget[0].setHorizontalHeaderLabels(["Acceleration"])
        self.table_widget[1].setHorizontalHeaderLabels(["Angularvelocity"])
        self.table_widget[2].setHorizontalHeaderLabels(["Angle"])
        self.table_widget[3].setHorizontalHeaderLabels(["Mag"])
        
        self.SetImuTableValue(self.table_widget[0],self.AccelerationX,self.AccelerationY,self.AccelerationZ,self.Temperature)
        self.SetImuTableValue(self.table_widget[1],self.AngularvelocityX,self.AngularvelocityY,self.AngularvelocityZ,self.Voltage)
        self.SetImuTableValue(self.table_widget[2],self.Roll,self.Pitch,self.Yaw,self.WT61P_ver)
        
        self.is_reconnecting = False        

        # Set up a QTimer
        self.timer = QTimer()
        self.timer.setInterval(100)  # Set timer to trigger every 1000ms (1 second)
        self.timer.timeout.connect(self.on_timer_timeout)  # Connect timeout to the handler function
       

        self.IMUtimer = QTimer()
        self.IMUtimer.setInterval(100)  # Set timer to trigger every 1000ms (1 second)
        self.IMUtimer.timeout.connect(self.on_IMU_timer_timeout)  # Connect timeout to the handler function
       
        self.RpLidartimer = QTimer()
        self.RpLidartimer.setInterval(50)  # Set timer to trigger every 1000ms (1 second)
        self.RpLidartimer.timeout.connect(self.on_RpLidar_timer_timeout)  # Connect timeout to the handler function


        self.MotorTimer = QTimer()
        self.MotorTimer.setInterval(50)  # Set timer to trigger every 1000ms (1 second)
        self.MotorTimer.timeout.connect(self.on_Motor_timer_timeout)  # Connect timeout to the handler function

        self.XmodemValueInitialize()

        global uartImustate,uartImuCnt
        uartImustate = IDLE_S
        uartImuCnt   = 0
        self.motors = ZLAC8015D.Controller()
        self.Motorperiod     = 0
        self.last_time = time.time()
        # debug console
        self.textEdit = QTextEdit(self)
        self.textEdit.setReadOnly(True)  # Make it read-only to prevent editing
        self.textEdit.setFixedSize(800, 90)
        
        self.ConnectLabel = [QLabel('MCU', self),QLabel('IMU', self),QLabel('MOT', self),QLabel('RP',self)]       
        for i in range(4):
            self.ConnectLabel[i].setFixedSize(100, 15)
            # if (i%2 == 0):
            self.ConnectLabel[i].move(620+110* int(1 if(i%2)==0 else 0) , Ypos +(int(1 if(i>1) else 0))* 16)
            # self.ConnectLabel[i].move(620+150*(int(0 if(i%2)==0 else 1)) , Ypos + int(1 if(i%2)==0 else 0 * 16))
            # else:
            #     self.ConnectLabel[i].move(620 , Ypos + (i/2 * 16))
        self.USLabel = [QLabel('US1:', self),QLabel('US2:', self),QLabel('US3:', self),QLabel('US4:',self),QLabel('US5:', self),QLabel('US6:', self),QLabel('US7:', self),QLabel('US8:',self)]        
        self.SWLabel = [QLabel('BotSW:', self),QLabel('TopSW:', self),QLabel('PowerSW:', self)]        
        self.BatteryLabel = QLabel('Battery: 0.0V', self)
        self.VerLabel   = [QLabel('B:', self),QLabel('IMU:', self),QLabel('RP:', self),QLabel('T1:', self),QLabel('T2:', self),QLabel('T3:', self),QLabel('T4:', self)]
        self.MotorLabel = [QLabel('P:', self),QLabel('LEFT:', self),QLabel('RIGHT:', self)]
        for i in range(8):
            self.USLabel[i].setFixedSize(80, 15)        
            #self.ConnectLabel[i].move(620+110* int(1 if(i%2)==0 else 0) , Ypos +(int(1 if(i>1) else 0))* 16)
        for i in range(3):
            self.SWLabel[i].setFixedSize(80, 15)    
            self.MotorLabel[i].setFixedSize(150, 15)
            self.ChangeLableColor(self.SWLabel[i],0)
            self.ChangeLableColor(self.MotorLabel[i],1)

            

        self.BatteryLabel.setFixedSize(150, 15)

        Ypos += 30
    
        self.FindSerialPortInfo()
        
        Ypos += 5
        
        for i in range(7):
            self.VerLabel[i].setFixedSize(150, 15)
            if i < 3:
                self.VerLabel[i].move(10+152*i, Ypos)
            else :
                self.VerLabel[i].move(10+152*(i-3), Ypos+16)

            self.ChangeLableColor(self.VerLabel[i],0)

        self.VerLabel[2].setFixedSize(151*3, 15)

        Ypos += 20
        # system version 
        # self.VerBaseLabel1 = QLabel('B :', self)        
        # self.VerTrayLabel1 = QLabel('T1 :', self)
        # self.VerTrayLabel2 = QLabel('T2 :', self)
        # self.VerTrayLabel3 = QLabel('T3 :', self)
        # self.VerTrayLabel4 = QLabel('T4 :', self)
        

        # self.VerLabel[0].setFixedSize(150, 15)
        # self.VerTrayLabel1.setFixedSize(150, 15)
        # self.VerTrayLabel2.setFixedSize(150, 15)
        # self.VerTrayLabel3.setFixedSize(150, 15)
        # self.VerTrayLabel4.setFixedSize(150, 15)

        # system version 
        # Ypos += 20
        # self.VerBaseLabel1.move(10, Ypos)
        # self.VerTrayLabel1.move(160, Ypos)
        # self.VerTrayLabel2.move(320, Ypos)
        # self.VerTrayLabel3.move(480, Ypos)
        # self.VerTrayLabel4.move(640, Ypos)



        Ypos += 30
        # Horizontal layout for u8Ledflag and u8LedTrayflag on the same line
        u8LedflagLabel = QLabel("Dat 1:", self)
        u8LedflagLabel.setFixedSize(50, 30)
        u8LedflagLabel.move(1010,Ypos)
        self.u8LedflagEdit = QTextEdit(self)
        self.u8LedflagEdit.setPlaceholderText("Enter value (0-255)")
        self.u8LedflagEdit.setFixedSize(50, 30)
        self.u8LedflagEdit.move(1045,Ypos)        
        u8LedTrayflagLabel = QLabel("Dat 2:", self)
        u8LedTrayflagLabel.setFixedSize(50, 30)
        u8LedTrayflagLabel.move(1110,Ypos)
        self.u8LedTrayflagEdit = QTextEdit(self)
        self.u8LedTrayflagEdit.setPlaceholderText("Enter value (0-255)")
        self.u8LedTrayflagEdit.setFixedSize(50, 30)
        self.u8LedTrayflagEdit.move(1145,Ypos)
        self.u8LedflagEdit.setText(f"1")
        self.u8LedTrayflagEdit.setText(f"0")        
        self.label5_Cmd = QLabel("Select CMD", self)
        self.label5_Cmd.setFixedSize(80, 30)
        self.label5_Cmd.move(1010,Ypos+35)
        self.combo_box_Cmd = QComboBox(self)
        self.combo_box_Cmd.addItems(['Scan','Stop'])
        self.combo_box_Cmd.setFixedSize(100, 30)        
        self.sendButton_Cmd = QPushButton('Send Cmd', self)
        self.sendButton_Cmd.clicked.connect(self.SendCmd)
        self.sendButton_Cmd.setFixedSize(100, 30)
        self.sendButton_Cmd.setEnabled(True)
        
         # 선택된 항목을 표시할 라벨
        self.DownloadcheckBox = QCheckBox('UseFwDownload',self)
        self.LEDctrl          = QCheckBox('LED Control',self)        
        self.TrayUpdate       = QCheckBox('TraySensor Update',self)        
        self.IMUcheckBox      = QCheckBox('IMU Info',self)        
        self.ReportUpdate     = QCheckBox('Send Report',self)
        self.MotorCheckBox    = QCheckBox('Motor Move',self)
        self.RPLidarCheck     = QCheckBox('Lidar',self)
        
        self.DownloadcheckBox.setFixedSize(150, 30)        
        self.LEDctrl.setFixedSize(150, 30)
        self.TrayUpdate.setFixedSize(150, 30)
        self.IMUcheckBox.setFixedSize(150, 30)        
        self.ReportUpdate.setFixedSize(150, 30)
        self.MotorCheckBox.setFixedSize(150, 30)        
        self.RPLidarCheck.setFixedSize(150, 30)       

        self.DownloadcheckBox.stateChanged.connect(self.on_Downloadcheckbox_state_changed)       
        self.ReportUpdate.stateChanged.connect(self.on_ReportUpdate_state_changed)
        self.IMUcheckBox.stateChanged.connect(self.on_IMU_state_changed)
        self.LEDctrl.stateChanged.connect(self.on_LEDctrl_state_changed)
        self.TrayUpdate.stateChanged.connect(self.on_TrayUpdate_state_changed)
        self.MotorCheckBox.stateChanged.connect(self.on_Mot_state_changed)
        self.RPLidarCheck.stateChanged.connect(self.on_Lidar_state_changed)
        
        # on_Mot_state_changed
        Ypos+=20
        Xpos = 10
        self.DownloadcheckBox.move(Xpos,Ypos-35)        
        self.ReportUpdate.move(Xpos,Ypos)
        Xpos += 210        
        self.IMUcheckBox.move(Xpos,Ypos-35)        
        self.LEDctrl.move(Xpos,Ypos)                
        Xpos += 210        
        self.MotorCheckBox.move(Xpos,Ypos-35)        
        self.TrayUpdate.move(Xpos,Ypos)        
        Xpos += 210
        self.RPLidarCheck.move(Xpos,Ypos-35)  

        self.DP_POS_Y = Ypos+40
        self.TrayUpdateCombo = QComboBox(self)
        self.TrayUpdateCombo.addItems(['Tray1', 'Tray2', 'Tray3', 'Tray4'])
        self.TrayUpdateCombo.setFixedSize(140,30)
        self.TrayUpdateCombo.setCurrentIndex(0)
        # Ypos = 150 + 20 + 35
        print("DP_POS_Y " + str(self.DP_POS_Y))
        # LED Control 
        Ypos += 30
        # 라디오 버튼 생성
        self.FBLabel = QLabel("Front/Back", self)
        self.FBLabel.setFixedSize(80, 30)
        
        self.UVLabel = QLabel("UV LAMP", self)
        self.UVLabel.setFixedSize(80, 30)
        
        self.Base_radio_button1 = QRadioButton("Off"     ,self)
        self.Base_radio_button2 = QRadioButton("On"      ,self)
        self.Base_radio_button3 = QRadioButton("Toggle"  ,self)
        self.Base_fb_button_group = QButtonGroup(self)

        self.Base_radio_button1.setFixedSize(50,30)
        self.Base_radio_button2.setFixedSize(50,30)
        self.Base_radio_button3.setFixedSize(100,30)                

        self.Base_fb_button_group.addButton(self.Base_radio_button1,0)
        self.Base_fb_button_group.addButton(self.Base_radio_button2,1)
        self.Base_fb_button_group.addButton(self.Base_radio_button3,2)        
        self.uv_radio_button1 = QRadioButton("Off"     ,self)
        self.uv_radio_button2 = QRadioButton("On"      ,self)
        self.uv_radio_button3 = QRadioButton("Toggle"  ,self)
        self.uv_fb_button_group = QButtonGroup(self)
        self.uv_radio_button1.setFixedSize(50,30)
        self.uv_radio_button2.setFixedSize(50,30)
        self.uv_radio_button3.setFixedSize(100,30)        
       
        self.uv_fb_button_group.addButton(self.uv_radio_button1,0)
        self.uv_fb_button_group.addButton(self.uv_radio_button2,4)
        self.uv_fb_button_group.addButton(self.uv_radio_button3,8)                

        self.T1Label = QLabel("TRAY(1)LED", self)
        self.T1Label.setFixedSize(80, 30)
        
        self.T2Label = QLabel("TRAY(2)LED", self)
        self.T2Label.setFixedSize(80, 30)
                
        self.tray1_radio_button1 = QRadioButton("Off"     ,self)
        self.tray1_radio_button2 = QRadioButton("On"      ,self)
        self.tray1_radio_button3 = QRadioButton("Toggle"  ,self)
        self.tray1_button_group = QButtonGroup(self)
        self.tray1_radio_button1.setFixedSize(50,30)
        self.tray1_radio_button2.setFixedSize(50,30)
        self.tray1_radio_button3.setFixedSize(100,30)
        
        self.tray1_button_group.addButton(self.tray1_radio_button1,0)
        self.tray1_button_group.addButton(self.tray1_radio_button2,1)
        self.tray1_button_group.addButton(self.tray1_radio_button3,2)

        self.tray2_radio_button1 = QRadioButton("Off"     ,self)
        self.tray2_radio_button2 = QRadioButton("On"      ,self)
        self.tray2_radio_button3 = QRadioButton("Toggle"  ,self)
        self.tray2_button_group = QButtonGroup(self)
        self.tray2_radio_button1.setFixedSize(50,30)
        self.tray2_radio_button2.setFixedSize(50,30)
        self.tray2_radio_button3.setFixedSize(100,30)
       
        self.tray2_button_group.addButton(self.tray2_radio_button1,0)
        self.tray2_button_group.addButton(self.tray2_radio_button2,4)
        self.tray2_button_group.addButton(self.tray2_radio_button3,8)        
        
        self.T3Label = QLabel("TRAY(3)LED", self)
        self.T3Label.setFixedSize(80, 30)        
        self.T4Label = QLabel("TRAY(4)LED", self)
        self.T4Label.setFixedSize(80, 30)       
        self.tray3_radio_button1 = QRadioButton("Off"     ,self)
        self.tray3_radio_button2 = QRadioButton("On"      ,self)
        self.tray3_radio_button3 = QRadioButton("Toggle"  ,self)
        self.tray3_button_group = QButtonGroup(self)
        self.tray3_radio_button1.setFixedSize(50,30)
        self.tray3_radio_button2.setFixedSize(50,30)
        self.tray3_radio_button3.setFixedSize(100,30)
        
        self.tray3_button_group.addButton(self.tray3_radio_button1,0)
        self.tray3_button_group.addButton(self.tray3_radio_button2,16)
        self.tray3_button_group.addButton(self.tray3_radio_button3,32)

        self.tray4_radio_button1 = QRadioButton("Off"     ,self)
        self.tray4_radio_button2 = QRadioButton("On"      ,self)
        self.tray4_radio_button3 = QRadioButton("Toggle"  ,self)
        self.tray4_button_group = QButtonGroup(self)
        self.tray4_radio_button1.setFixedSize(50,30)
        self.tray4_radio_button2.setFixedSize(50,30)
        self.tray4_radio_button3.setFixedSize(100,30)
       
        self.tray4_button_group.addButton(self.tray4_radio_button1,0)
        self.tray4_button_group.addButton(self.tray4_radio_button2,16)
        self.tray4_button_group.addButton(self.tray4_radio_button3,32)

        self.Base_radio_button1.setChecked(True)
        self.uv_radio_button1.setChecked(True)
        self.tray1_radio_button1.setChecked(True)
        self.tray2_radio_button1.setChecked(True)
        self.tray3_radio_button1.setChecked(True)
        self.tray4_radio_button1.setChecked(True)

        self.Base_fb_button_group.buttonClicked.connect(self.read_selected_index)
        self.uv_fb_button_group.buttonClicked.connect(self.read_selected_index)
        self.tray1_button_group.buttonClicked.connect(self.read_selected_index)
        self.tray2_button_group.buttonClicked.connect(self.read_selected_index)
        self.tray3_button_group.buttonClicked.connect(self.read_selected_index)
        self.tray4_button_group.buttonClicked.connect(self.read_selected_index)
        
        Ypos1 = Ypos
        # ----------------------------------------------------------------
        # position settings
               
        self.combo_box_Cmd.setCurrentIndex(0)

        layout_3dplot = QHBoxLayout()
        self.canvases = []        
        self.canvas = Bar3DCanvas(self)            
        self.canvas.setFixedSize(580,300)    
        self.canvas.move(1524,Ypos) 
        self.canvases.append(self.canvas)
        layout_3dplot.addWidget(self.canvas)  # Arrange in a grid (2 columns)         
        self.array_signal = ArraySignal() # Create an instance of ArraySignal        
        self.array_signal.updated.connect(self.update_all_plots) # Connect the signal to the canvases' update function        
        self.data = [np.random.randint(0, 1, size=(4, 4))] # Initial data for each graph        
        for self.canvas, data in zip(self.canvases, self.data):
            self.canvas.update_plot(data)  # Plot initial data
        
        # BarGraphCanvas 추가
        # 초기 데이터 설정
        self.data_bar = np.random.randint(1, 2000, size=17)  # 초기 데이터 17개
        self.canvas_bar = BarGraphCanvas(self)
        layout_3dplot.addWidget(self.canvas_bar)
        self.canvas_bar.setFixedSize(580,300)   
        # self.canvas_bar.move(580,300) 
        # 초기 데이터로 그래프 표시
        self.canvas_bar.plot_bar_graph(self.data_bar)
        self.array_signal_bar = ArraySignal() # Create an instance of ArraySignal        
        self.array_signal_bar.updated.connect(self.bar_update_data) # Connect the signal to the canvases' update function  

        self.textEditTof = QTextEdit(self)
        self.textEditTof.setReadOnly(True)  # Make it read-only to prevent editing
        self.textEditTof.setFixedSize(580, 25)

        # TOF 
        self.onShowCtrlBox('default') 
        # main_layout.addWidget(self.progress_bar)
        # self.setLayout(main_layout)            
        #find send files
        file_name = find_and_read_files(pattern="*Base_*.bin", directory=".")    
        # current_dir = os.path.dirname(os.path.abspath(__file__))
        # file_name = os.path.join(current_dir, "example.txt")
        # file_name = 'D:/hskim_work/1.idro/3.fw/MCU_FW_code/06.Tray_TOF_for_Roxi_with_nuvoton/Project/LIDRO-BOARD/LIDRO-BOARD-TOF_V0/Objects/Bin/aPP/TOF_MCU_V0.3.bin'
        if file_name != None :
            with open(file_name, 'rb') as file:
                FwBaseBin = file.read()       
            self.fileLabel1.setText(f"Base: {os.path.basename(file_name)}")  # Show selected file 1 name
            print("Base File Size",len(FwBaseBin)) 

        file_name = find_and_read_files(pattern="*TOF_MCU_*.bin", directory=".")    
        if file_name != None :
            with open(file_name, 'rb') as file:
                FwTrayBin = file.read()       
            self.fileLabel2.setText(f"Tray: {os.path.basename(file_name)}")  # Show selected file 1 name
            print("Tray File Size",len(FwTrayBin)) 
        #'D:/hskim_work/1.idro/3.fw/MCU_FW_code/06.Tray_TOF_for_Roxi_with_nuvoton/Project/LIDRO-BOARD/LIDRO-BOARD-TOF_V0/Objects/Bin/ROX_MAIN_Base.bin'
        # Set the layout for the window            
        print(Tray_TofSensor)
        
        global verdateTime,mucversion
                # Set window title and size
        self.setWindowTitle('roxi factory tools v' + mucversion + verdateTime)
        self.setGeometry(100, 100, 800, 600)
        # self.toggle_connection()


        self.progress_bar.setVisible(False) 
        self.sendButton_Cmd.setEnabled(False)

        
        self.DownloadcheckBox.setChecked(False)
        self.progress_bar.setVisible(False) 
        self.sendButton_Cmd.setEnabled(True)
        # Get System version 
        if self.serial_port and self.serial_port.isOpen():                                           
                self.GetMCU_VERSION_INFO()
                print('Get MCU version') 

        if self.RPlidarserial_port and self.RPlidarserial_port.isOpen():                                           
                self.RpLidarSendCmdWithOutPayload(SL_LIDAR_CMD_GET_DEVICE_INFO)
                
    def FindSerialPortInfo(self):
        if(self.update_serial_ports("ttyUSB0",self.comboMCU) == False):
            self.ConnectLabel[0].setStyleSheet("background-color: red; color: black;")
            self.ConnectLabel[0].setText('MCU :Error')
        else:
            self.ConnectLabel[0].setStyleSheet("background-color: green; color: black;")
            self.ConnectLabel[0].setText('MCU :Found')
            self.toggle_connection()

        if(self.update_serial_ports("ttyUSB1",self.comboTRAY) == False):
            print('TRAY COM :Error')
        else:
            print('TRAY COM :Found')
            
            self.Traytoggle_connection()

        if(self.update_serial_ports("ttyUSB2",self.comboIMU) == False):
            self.ConnectLabel[1].setStyleSheet("background-color: red; color: black;")
            self.ConnectLabel[1].setText('IMU :Error')
        else:
            self.ConnectLabel[1].setStyleSheet("background-color: green; color: black;")
            self.ConnectLabel[1].setText('IMU :Found')
            self.IMUtoggle_connection()

        if(self.update_serial_ports("ttyUSB3",self.comboMOT) == False):
            self.ConnectLabel[2].setStyleSheet("background-color: red; color: black;")
            self.ConnectLabel[2].setText('MOT : Error')
        else:
            self.ConnectLabel[2].setStyleSheet("background-color: green; color: black;")
            self.ConnectLabel[2].setText('MOT : Found')
            self.MOTtoggle_connection()

        if(self.update_serial_ports("ttyUSB4",self.comboRPLIDAR) == False):
            self.ConnectLabel[3].setStyleSheet("background-color: red; color: black;")
            self.ConnectLabel[3].setText('RP : Error')
        else:
            self.ConnectLabel[3].setStyleSheet("background-color: green; color: black;")
            self.ConnectLabel[3].setText('RP : Found')
            self.RPtoggle_connection()
    def SetImuTableValue(self,table_widget,d1,d2,d3,d4):
        item = QTableWidgetItem(f"{d1:.3f}")
        item.setTextAlignment(Qt.AlignRight)  # 텍스트 왼쪽 정렬
        table_widget.setItem(0, 1, item)
        item = QTableWidgetItem(f"{d2:.3f}")
        item.setTextAlignment(Qt.AlignRight)  # 텍스트 왼쪽 정렬
        table_widget.setItem(1, 1, item)
        item = QTableWidgetItem(f"{d3:.3f}")
        item.setTextAlignment(Qt.AlignRight)  # 텍스트 왼쪽 정렬
        table_widget.setItem(2, 1, item)
        item = QTableWidgetItem(f"{d4:.3f}")
        item.setTextAlignment(Qt.AlignRight)  # 텍스트 왼쪽 정렬
        table_widget.setItem(3, 1, item)

    def bar_external_update(self,new_data):        
        self.array_signal_bar.updated.emit(new_data)  # Emit signal with updated arrays
    def bar_update_data(self,data_bar):        
        self.canvas_bar.plot_bar_graph(data_bar) # 그래프 업데이트
    
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
    
    def ImuPosInfo(self,Ypos,Xpos):
        YposT = Ypos
        XposT = Xpos
        for i in range(4):
            self.table_widget[i].move(XposT,YposT)
            item = QTableWidgetItem(f"X:")
            self.table_widget[i].setItem(0, 0, item)
            item = QTableWidgetItem(f"Y:")
            self.table_widget[i].setItem(1, 0, item)
            item = QTableWidgetItem(f"Z:")
            self.table_widget[i].setItem(2, 0, item)
            XposT += 256
        return YposT + 180
    def Lidar_Info(self,Ypos,Xpos):
        # Ypos += 150     
        self.combo_box_Cmd.move(Xpos,Ypos)
        self.sendButton_Cmd.move(Xpos+100,Ypos) 
        return Ypos + 40
    def Mot_Info(self,Ypos,Xpos):
        # Ypos += 150     

        for i in range(3):
            self.MotorLabel[i].move(Xpos+152*i, Ypos)

        return Ypos + 25
    def TOF_Info(self,Ypos,Xpos):  
        self.TrayUpdateCombo.move(Xpos+200,Ypos)
        Ypos += 40     
        #
        self.canvas_bar.move(Xpos,Ypos)
        Ypos += 300 + 5
        self.textEditTof.move(Xpos,Ypos)             
        return Ypos + 25
    def UpdateInfo(self,Ypos,Xpos):
        
        Xpos = 10
        self.fileLabel1.move(Xpos,Ypos)
        self.openButton1.move(Xpos + 180,Ypos)
        self.label5.move(Xpos + 310,Ypos)
        self.sendButton.move(Xpos + 450,Ypos)  
        Ypos += 40                    
        self.fileLabel2.move(10,Ypos)
        self.openButton2.move(Xpos + 180,Ypos)
        self.combo_box.move(Xpos + 310,Ypos)
        Ypos += 150
        self.progress_bar.move(Xpos,Ypos)
        return Ypos + 30
    def LED_ctrlPosition(self,Ypos,Xpos):
        
        self.FBLabel.move(10,Ypos)
        self.UVLabel.move(310,Ypos)
        Xpos = 100
        self.Base_radio_button1.move(Xpos,Ypos)
        self.Base_radio_button2.move(Xpos+50,Ypos)
        self.Base_radio_button3.move(Xpos+50*2,Ypos)       
        Xpos += 300
        self.uv_radio_button1.move(Xpos,Ypos)
        self.uv_radio_button2.move(Xpos+50,Ypos)
        self.uv_radio_button3.move(Xpos+50*2,Ypos)
        Ypos += 32
        Xpos = 100
        self.T1Label.move(10,Ypos)
        self.T2Label.move(310,Ypos)
        Xpos = 100
        self.tray1_radio_button1.move(Xpos,Ypos)
        self.tray1_radio_button2.move(Xpos+50,Ypos)
        self.tray1_radio_button3.move(Xpos+50*2,Ypos)
        Xpos += 300
        self.tray2_radio_button1.move(Xpos,Ypos)
        self.tray2_radio_button2.move(Xpos+50,Ypos)
        self.tray2_radio_button3.move(Xpos+50*2,Ypos)        
        Ypos += 32
        Xpos = 100
        self.T3Label.move(10,Ypos)
        self.T4Label.move(310,Ypos)
        Xpos = 100
        self.tray3_radio_button1.move(Xpos,Ypos)
        self.tray3_radio_button2.move(Xpos+50,Ypos)
        self.tray3_radio_button3.move(Xpos+50*2,Ypos)
       
        Xpos += 300
        self.tray4_radio_button1.move(Xpos,Ypos)
        self.tray4_radio_button2.move(Xpos+50,Ypos)
        self.tray4_radio_button3.move(Xpos+50*2,Ypos)
        
        return Ypos +32
    def ChangeLableColor(self,ColorObject,value):
        if (int(value)) == 0:
            ColorObject.setStyleSheet("background-color: red; color: black;")
        else:
            ColorObject.setStyleSheet("background-color: green; color: black;")

    def uart_process(self,cc):
        global uart5state, uartTrayCnt, cTofBuff,Xmodemstate
        
        # print(f"{cc:02x},{uart5state:02x}",end =' ')
        print(f"{cc:02x}",end =' ')
        # print(type(cc))
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
            # cTofBuff[uartTrayCnt] = cc
            # uartTrayCnt += 1
            if uartTrayCnt >= (cTofBuff[P_LEN] + 2):
                uart5state = CRC_S  # Move to CRC state

        elif uart5state == CRC_S:
            # cTofBuff[uartTrayCnt] = cc
            # uartTrayCnt += 1
            if uartTrayCnt >= (cTofBuff[P_LEN] + 4):
                uart5state = ETX_S  # Move to ETX state

        elif uart5state == ETX_S:
            uart5state = IDLE_S  # Reset state to IDLE                
            if cc == ETX:
                # print(f"------>{uartTrayCnt},{uartTrayCnt:02x}")
                # Calculate CRC and verify it
                crc_dat1 = crc16(cTofBuff , uartTrayCnt - 2)
                crc_dat2 = (cTofBuff[uartTrayCnt - 1] << 8) | cTofBuff[uartTrayCnt - 2]
                # print(cTofBuff[:10],"%x"% crc_dat1,"%x"% crc_dat2)
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
                        self.ChangeLableColor(self.VerLabel[0],ver_year[0])
                        self.ChangeLableColor(self.VerLabel[3],ver_year[1])
                        self.ChangeLableColor(self.VerLabel[4],ver_year[2])
                        self.ChangeLableColor(self.VerLabel[5],ver_year[3])
                        self.ChangeLableColor(self.VerLabel[6],ver_year[4])
                        
                    
                        self.VerLabel[0].setText(f"B: V{ver_major[0]}.{ver_minor[0]}({ver_year[0]}.{ver_month[0]}.{ver_date[0]})")
                        self.VerLabel[3].setText(f"T1 : V{ver_major[1]}.{ver_minor[1]}({ver_year[1]}.{ver_month[1]}.{ver_date[1]})")
                        self.VerLabel[4].setText(f"T2 : V{ver_major[2]}.{ver_minor[2]}({ver_year[2]}.{ver_month[2]}.{ver_date[2]})")
                        self.VerLabel[5].setText(f"T3: V{ver_major[3]}.{ver_minor[3]}({ver_year[3]}.{ver_month[3]}.{ver_date[3]})")
                        self.VerLabel[6].setText(f"T4: V{ver_major[4]}.{ver_minor[4]}({ver_year[4]}.{ver_month[4]}.{ver_date[4]})")
                        
                    elif(u8CmdSetTrayBaseLedCtrl == cTofBuff[3]):

                        print("u8CmdSetTrayBaseLedCtrl")
                    elif(u8CmdSetReportSensor == cTofBuff[3]):

                        print("u8CmdSetReportSensor")        
                    elif(SendReport == cTofBuff[3]):

                        
                        for i in range(8):
                            if int(cTofBuff[5+i*2] <<8 | cTofBuff[4+i*2]) == 0 :
                               self.USLabel[i].setStyleSheet("background-color: red; color: black;")
                            else:
                                self.USLabel[i].setStyleSheet("background-color: green; color: black;")   
                            self.USLabel[i].setText(f"US{(i+1)}: {int(cTofBuff[5+i*2] <<8 | cTofBuff[4+i*2])}")                           
                            #self.ConnectLabel[i].move(620+110* int(1 if(i%2)==0 else 0) , Ypos +(int(1 if(i>1) else 0))* 16)
                        if (int(cTofBuff[24]) & 1) != 0:
                            self.SWLabel[0].setText("BotSW: ON")
                            self.SWLabel[0].setStyleSheet("background-color: red; color: black;")
                        else:
                            self.SWLabel[0].setText("BotSW: OFF")
                            self.SWLabel[0].setStyleSheet("background-color: green; color: black;")
                        
                        if (int(cTofBuff[24]) & 4) != 0:
                            self.SWLabel[1].setText("TopSW: ON")
                            self.SWLabel[1].setStyleSheet("background-color: red; color: black;")
                        else:
                            self.SWLabel[1].setText("TopSW: OFF")
                            self.SWLabel[1].setStyleSheet("background-color: green; color: black;")
                        
                        if (int(cTofBuff[24]) & 2) != 0:
                            self.SWLabel[2].setText("PowerSW: ON")
                            self.SWLabel[2].setStyleSheet("background-color: red; color: black;")
                        else:
                            self.SWLabel[2].setText("PowerSW: OFF")
                            self.SWLabel[2].setStyleSheet("background-color: green; color: black;")

                        if int(cTofBuff[5+i*2] <<8 | cTofBuff[4+i*2]) == 0 :
                            self.USLabel[i].setStyleSheet("background-color: red; color: black;")
                        else:
                            self.USLabel[i].setStyleSheet("background-color: green; color: black;")   

                        if (int(cTofBuff[21] <<8 | cTofBuff[20])) != 0:
                            self.BatteryLabel.setStyleSheet("background-color: red; color: black;")
                        else:
                            self.BatteryLabel.setStyleSheet("background-color: green; color: black;")

                        self.BatteryLabel.setText(f"Battery: {int(cTofBuff[21] <<8 | cTofBuff[20])}V")
                        # self.USLabel = [QLabel('US1:', self),QLabel('US2:', self),QLabel('US3:', self),QLabel('US4:',self),QLabel('US5:', self),QLabel('US6:', self),QLabel('US7:', self),QLabel('US8:',self)]        
                        # self.SWLabel = [QLabel('BotSW:', self),QLabel('TopSW:', self),QLabel('PowerSW:', self)]        
                        # self.SWLabel = QLabel('Battery: 0.0V')
                        
                        
                        # send =              
                        self.textEdit.append(f"US1({int(cTofBuff[5] <<8 | cTofBuff[4])}),US2({int(cTofBuff[7] <<8 | cTofBuff[6])}),US3({int(cTofBuff[9] <<8 | cTofBuff[8])}),"
                                             f"US4({int(cTofBuff[11] <<8 | cTofBuff[10])}),US5({int(cTofBuff[13] <<8 | cTofBuff[12])}),US6({int(cTofBuff[15] <<8 | cTofBuff[14])}),"
                                             f"US7({int(cTofBuff[17] <<8 | cTofBuff[16])}),US8({int(cTofBuff[19] <<8 | cTofBuff[18])})"
                                             f",BatteryVoltage({int(cTofBuff[21] <<8 | cTofBuff[20])}),SW({int(cTofBuff[24])})")


                        print("SendReport")                             
                    elif(GetTRAYSensorDATA == cTofBuff[3]):
                        global Tray_TofSensor,Tray_UsSensor
                        # Tray_TofSensor = [[0 for _ in range(4)] for _ in range(16)]
                        # Tray_UsSensor = [0 for _ in range(4)]
                        startPos = P_Dat
                        for i in range(4):
                            Tray_UsSensor[i]  = (cTofBuff[startPos + 1] << 8) | (cTofBuff[startPos + 0])
                            Tray_TofTotalSensor[i][0] = Tray_UsSensor[i]
                            # print(f"\n {cTofBuff[startPos + 1]:02x} {cTofBuff[startPos + 0]:02x} " ,end = ':')
                            startPos = startPos + 2
                            for j in range(16):
                                Tray_TofSensor[i][j] = (cTofBuff[startPos + j*2 + 1] << 8) | (cTofBuff[startPos + j*2+0])
                                Tray_TofTotalSensor[i][j+1] = Tray_TofSensor[i][j]
                                # print(f"{cTofBuff[startPos + j*2 + 0]:02x} {cTofBuff[startPos + j*2 + 1]:02x} " ,end = ',')
                            startPos = startPos + 32

                    
                        self.textEditTof.append(f"Status:{Tray_TofSensor[self.TrayUpdateCombo.currentIndex()]}")
                        self.simulate_external_update(Tray_TofSensor[self.TrayUpdateCombo.currentIndex()])
                        self.bar_external_update(Tray_TofTotalSensor[self.TrayUpdateCombo.currentIndex()])

                        if self.TrayUpdate.isChecked():
                            self.Get_TraySensorDat()

                    # print(f"Recive done {cTofBuff[1]:02x} {cTofBuff[2]:02x} {cTofBuff[3]:02x} {cTofBuff[4]:02x}")   # Aleks
                else:
                    if u8Uart_TOFDebugPrintflag == 2:
                        print(f"TOF CRC Error --> {cc:02x} : {crc_dat1:04x} {crc_dat2:04x}")
    def simulate_external_update(self,new_data):
        # Simulate an external data update (random values for all graphs)
        con_data = [np.array(new_data).reshape((4, 4))]        
        self.array_signal.updated.emit(con_data)  # Emit signal with updated arrays

    def update_all_plots(self, new_data):        
        # Update each graph with corresponding new data
        for canvas, data in zip(self.canvases, new_data):
            canvas.update_plot(data)
    def handle_error(self, error):
        """Handle serial port error (e.g., connection lost)"""
        if error == QSerialPort.ResourceError:            
            self.textEdit.append(f"Status: Connection lost!")
            print("Serial port connection lost!")
            self.toggle_connection()
            self.is_reconnecting = True
            # time.sleep(1)
            # self.update_serial_ports()
            # self.toggle_connection()
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
        # print(_bytes)
        # for i in len(_bytes):
        #     print(f"{_bytes[i]:02x}", end=". ")
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
                    # self.textEdit.append(f"{self.parse_device_info(cRpBuff[7:30])}") 
                    

                uartRpstate = IDLE_S
                uartRpCnt   = 0

    def ImuUartProcess(self, rxOneDat):
        global uartImustate,uartImuCnt, cImuBuff

        # print(f"{rxOneDat:02x},{uartImuCnt},{uartImustate} ",end =' ')        
        cImuBuff[uartImuCnt] = rxOneDat; 
        uartImuCnt += 1
        uartImuCnt &= 0xff
        if uartImustate == IDLE_S:
            if rxOneDat == 0x55:
                uartImustate            = WT61P_PROTOCOL_TYPE
                uartImuCnt              = 1
                cImuBuff[0] = rxOneDat
            else:                
                uartImuCnt   = 0
                uartImustate = IDLE_S 
        elif uartImustate == WT61P_PROTOCOL_TYPE:

            if rxOneDat == TypeAcceleration:                
                uartImustate = WT61P_PROTOCOL_DATA
                # print("TypeAcceleration    0x51 detected.")
            elif rxOneDat == TypeAngularvelocity:                
                uartImustate = WT61P_PROTOCOL_DATA
                # print("TypeAngularvelocity 0x52 detected.")
            elif rxOneDat == TypeAngle:
                # print("TypeAngle           0x53 detected.")
                uartImustate = WT61P_PROTOCOL_DATA
            else:

                # print("Invalid header type detected.")
                uartImustate = WT61P_PROTOCOL_DATA
                # uartImustate = IDLE_S
                # uartImuCnt   = 0
        elif uartImustate == WT61P_PROTOCOL_DATA:
            if uartImuCnt > 10:
                CHECKSUM = 0
                for i in range(0, 10):
                    CHECKSUM += cImuBuff[i]
                CHECKSUM &= 0xff
                # print(f"CheckSum: {CHECKSUM:02x} ReadCheckSum: {( cImuBuff[10]):02x}")

                if CHECKSUM == (cImuBuff[10]):
                    # print("pass :",end = '')
                    # print(f"CheckSum: {CHECKSUM:02x} ReadCheckSum: {(cImuBuff[10]):02x}")
                    d1 = c_int16(cImuBuff[3] <<8 | cImuBuff[2])
                    d2 = c_int16(cImuBuff[5] <<8 | cImuBuff[4])
                    d3 = c_int16(cImuBuff[7] <<8 | cImuBuff[6])
                    d4 = c_int16(cImuBuff[9] <<8 | cImuBuff[8])
                    self.ImuDataRxTimer = 0
                    if cImuBuff[1] == TypeAcceleration:                                    
                        # print("TypeAcceleration     :",end = '')
                        #Acceleration X=((AxH<<8)|AxL)/32768*16g (g is the acceleration of gravity, preferably 9.8m/s2)
                        #Acceleration Y=((AxH<<8)|AxL)/32768*16g (g is the acceleration of gravity, preferably 9.8m/s2)
                        #Acceleration Z=((AxH<<8)|AxL)/32768*16g (g is the acceleration of gravity, preferably 9.8m/s2)
                        #32768 * 96.38 + 36.53
                        # SUM=0x55+0x51+AxL+AxH+AyL+AyH+AzL+AzH+TL+Th
                        self.AccelerationX = (float(d1.value)) / 32768.0 * 16.0
                        self.AccelerationY = (float(d2.value)) / 32768.0 * 16.0
                        self.AccelerationZ = (float(d3.value)) / 32768.0 * 16.0
                        self.Temperature   = (float(d4.value)) / 32768.0 * 96.38 + 36.53

                        self.SetImuTableValue(self.table_widget[0],self.AccelerationX,self.AccelerationY,self.AccelerationZ,self.Temperature)

                        # print(f"Acc: \t X{self.AccelerationX:.3f},\t\t Y{self.AccelerationY:.3f},\t\t Z{self.AccelerationZ:.3f},\t\t Temperature: {self.Temperature:.3f}")
                        # self.textEdit.append(f"US1({int(cTofBuff[5] <<8 | cTofBuff[4])}),US2({int(cTofBuff[7] <<8 | cTofBuff[6])}),US3({int(cTofBuff[9] <<8 | cTofBuff[8])}),US4({int(cTofBuff[11] <<8 | cTofBuff[10])}),US5({int(cTofBuff[13] <<8 | cTofBuff[12])}),US6({int(cTofBuff[15] <<8 | cTofBuff[14])}),US7({int(cTofBuff[17] <<8 | cTofBuff[16])}),US8({int(cTofBuff[19] <<8 | cTofBuff[18])})")
                    elif cImuBuff[1] == TypeAngularvelocity:                                    
                        self.AngularvelocityX   = (float(d1.value)) / 32768.0*2000.0
                        self.AngularvelocityY   = (float(d2.value)) / 32768.0*2000.0
                        self.AngularvelocityZ   = (float(d3.value)) / 32768.0*2000.0
                        self.Voltage            = (float(d4.value)) / 100.0

                        self.SetImuTableValue(self.table_widget[1],self.AngularvelocityX,self.AngularvelocityY,self.AngularvelocityZ,self.Voltage)
                        # print(f"W: \tX{self.AngularvelocityX:.3f},\tY{self.AngularvelocityY:.3f},\tZ{self.AngularvelocityZ:.3f},\t Voltage{self.Voltage:.3f}")

                        # print("TypeAngularvelocity  :",end = '')
                    elif cImuBuff[1] == TypeAngle:
                        self.Roll           = (float(d1.value)) / 32768.0*180.0
                        self.Pitch          = (float(d2.value)) / 32768.0*180.0
                        self.Yaw            = (float(d3.value)) / 32768.0*180.0
                        self.WT61P_ver      = (cImuBuff[9] <<8 | cImuBuff[8])

                        self.ChangeLableColor(self.VerLabel[1],self.WT61P_ver)
                        self.VerLabel[1].setText(f"IMU:"+f"{self.WT61P_ver} ")

                        self.SetImuTableValue(self.table_widget[2],self.Roll,self.Pitch,self.Yaw,self.WT61P_ver)
                        # print(f"Angle: \t X{self.Roll:.3f},\t Y{self.Pitch:.3f},\t Z{self.Yaw:.3f},\t WT61P_ver{self.WT61P_ver:.3f}")
                    # else:
                    #     print("????                 :")
                # print(f"Received WT61 data: {cImuBuff[0:10]}")
                # for i in range(0,12): 
                #     OneByte = int.from_bytes(bytes(cImuBuff[i:i+1]), byteorder='big') 
                #     print(f"{OneByte:02x}",end =' ')
                # print(f"<--Received WT61 data")
                uartImustate = IDLE_S
                uartImuCnt   = 0
    
    def MotorErrorCode(self,Code):
        NO_FAULT = 0x0000
        OVER_VOLT = 0x0001
        UNDER_VOLT = 0x0002
        OVER_CURR = 0x0004
        OVER_LOAD = 0x0008
        CURR_OUT_TOL = 0x0010
        ENCOD_OUT_TOL = 0x0020
        MOTOR_BAD = 0x0040
        REF_VOLT_ERROR = 0x0080
        EEPROM_ERROR = 0x0100
        WALL_ERROR = 0x0200
        HIGH_TEMP = 0x0400

        # 코드에 따른 상태 반환
        if Code == NO_FAULT:
            return "NO_FAULT"
        elif Code == OVER_VOLT:
            return "OVER_VOLT"
        elif Code == UNDER_VOLT:
            return "UNDER_VOLT"
        elif Code == OVER_CURR:
            return "OVER_CURR"
        elif Code == OVER_LOAD:
            return "OVER_LOAD"
        elif Code == CURR_OUT_TOL:
            return "CURR_OUT_TOL"
        elif Code == ENCOD_OUT_TOL:
            return "ENCOD_OUT_TOL"
        elif Code == MOTOR_BAD:
            return "MOTOR_BAD"
        elif Code == REF_VOLT_ERROR:
            return "REF_VOLT_ERROR"
        elif Code == EEPROM_ERROR:
            return "EEPROM_ERROR"
        elif Code == WALL_ERROR:
            return "WALL_ERROR"
        elif Code == HIGH_TEMP:
            return "HIGH_TEMP"
        else:
            return "UNKNOWN_FAULT"
        
    def on_Motor_timer_timeout(self):
        # This function is called every time the timer triggers
        self.MotorTimer.stop()
        if self.motors.client != None and self.motors.client.is_socket_open():
            rpmL, rpmR = self.motors.get_rpm()
            
            self.MotorLabel[0].setText(f"P: {self.Motorperiod}")
            self.MotorLabel[1].setText(f"LEFT:{rpmL})")
            self.MotorLabel[2].setText(f"RIGHT:{rpmR})")

            (L_fault_flag, L_fault_code), (R_fault_flag, R_fault_code) = self.motors.get_fault_code()
            if L_fault_flag == 0 and R_fault_flag == 0:
                print("period: {:.4f} rpmL: {:.1f} | rpmR: {:.1f}".format(self.Motorperiod,rpmL,rpmR))
                self.textEdit.append("period: {:.4f} rpmL: {:.1f} | rpmR: {:.1f}".format(self.Motorperiod,rpmL,rpmR))
                for i in range(3):
                    self.ChangeLableColor(self.MotorLabel[i],1)
            else:                
                errorInfo = ''
                if(L_fault_flag):
                    errorInfo = " left_Motor "+self.MotorErrorCode(L_fault_code)
                    self.ChangeLableColor(self.MotorLabel[1],0)
                    self.MotorLabel[1].setText("LEFT:"+self.MotorErrorCode(L_fault_code))
                if(R_fault_flag):
                    errorInfo += " Right_Motor "+self.MotorErrorCode(R_fault_code)
                    self.MotorLabel[2].setText("RIGHT:"+self.MotorErrorCode(R_fault_code))
                    self.ChangeLableColor(self.MotorLabel[2],0)
                print(errorInfo)
                self.textEdit.append(errorInfo)
            self.Motorperiod     = time.time() - self.last_time
            self.last_time  = time.time()
            if(self.MOTconnectButton.text() == 'MOT_Disconnect'):
                self.MotorTimer.start()         
        else:
            self.MotorLabel[0].setText("Not Connect MotorDriver")
            for i in range(3):
                self.ChangeLableColor(self.MotorLabel[i],0)

            port_name = self.comboMOT.currentText().split()[0] 
            self.textEdit.append(f"Mot 장치 관리자 포트({port_name}) RS485 설정 또는 연결은 확인해 주세요.") 
            self.MOTconnectButton.setText('MOT_Connect')
            self.MOTconnectButton.setStyleSheet("background-color: red; color: black;")
            self.motors.SerialportClosed()

            print("Motor is not connected")
            self.textEdit.append("Motor is not connected")
            
    def on_RpLidar_timer_timeout(self):
        # This function is called every time the timer triggers
        self.RpLidartimer.stop()
        # self.label.setText("Timer triggered!")  # Update the label or perform any action
        if self.RPlidarserial_port.isOpen():
            self.RpLidartimer.setInterval(50)
            # self.is_reconnecting = False
            #if self.serial_port.waitForReadyRead(100):  # Wait for data with a timeout of 100ms            
            data = self.RPlidarserial_port.readAll()  # Read available data            
            if data:
                    
                for i in range(0,data.size()): 
                    OneByte = int.from_bytes(bytes(data[i:i+1]), byteorder='big')                     
                    self.RpUartProcess(OneByte)
                    print(f"{OneByte:02x}",end =' ')
                # self.textEdit.append(f"Rx:{data}")   
                # uartImuCnt = 0
                # cImuBuff = bytearray(256)  # Adjust the buffer size if needed
                print(f"<-- RpRx")
            if(self.RPconnectButton.text() == 'RP_Disconnect'):
                self.RpLidartimer.start()

    def on_IMU_timer_timeout(self):
        # This function is called every time the timer triggers
        self.IMUtimer.stop()
        # self.label.setText("Timer triggered!")  # Update the label or perform any action
        if self.Imuserial_port.isOpen():
            self.IMUtimer.setInterval(100)

            if self.ImuDataRxTimer > 50 :
                if self.ImuDataRxTimer == 51:
                    self.textEdit.append(f"IMU Need Set Configration with comport({self.Imuserial_port.portName()})") 
                    self.ImuDataRxTimer += 1
                    print(self.Imuserial_port.baudRate())
                    popup = ImagePopup()
                    popup.exec()
            else :                
                self.ImuDataRxTimer += 1
                
            # self.is_reconnecting = False
            #if self.serial_port.waitForReadyRead(100):  # Wait for data with a timeout of 100ms            
            data = self.Imuserial_port.readAll()  # Read available data            
            if data:    
                for i in range(0,data.size()): 
                    OneByte = int.from_bytes(bytes(data[i:i+1]), byteorder='big') 
                    # print(f"{OneByte:02x}",end =' ')
                    self.ImuUartProcess(OneByte)
                # self.textEdit.append(f"Rx:{data}")   
                # uartImuCnt = 0
                # cImuBuff = bytearray(256)  # Adjust the buffer size if needed
            if self.ImuDataRxTimer != 0:
                print("IMU is not recived. Time "+str(self.ImuDataRxTimer)+"00ms",sep ='')

            if(self.IMUconnectButton.text() == 'IMU_Disconnect'):
                self.IMUtimer.start()

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
                    print(f"OneByte : {OneByte}")
                    if Xmodemstate != 0 :
                        
                        self.handle_xmodem(OneByte)
                        
                    else:
                        self.uart_process(OneByte)
                    # char = OneByte.to_bytes(1, byteorder='big')
                    # print(char)
            if(self.connectButton.text() == 'MCU_Disconnect'):
                self.timer.start()         

    def update_serial_ports(self, findPortName, ComboBox):
        ports = QSerialPortInfo.availablePorts()
        ComboBox.clear()
        index = -1

        for idx, port in enumerate(ports):
            description = port.description()
            port_display = f"{port.portName()} ({description})" if description else port.portName()

            if "ttyUSB" in port.portName():
                ComboBox.addItem(port_display)


            if findPortName in port.portName() or findPortName in port.serialNumber():
                # print("PortName:", port.portName())
                # print("Description:", port.description())
                # print("Manufacturer:", port.manufacturer())
                # print("SerialNumber:", port.serialNumber())
                # print("VendorIdentifier:", port.vendorIdentifier())
                # print("ProductIdentifier:", port.productIdentifier())
                index = idx

        if index != -1:
            ComboBox.setCurrentIndex(index)
            return True
        return False
    
    def on_port_selected(self):
        # Called when a new port is selected from the combo box
        if self.serial_port and self.serial_port.isOpen():
            self.serial_port.close()  # Close the currently opened port if any
    
    def IMUtoggle_connection(self):
        # Called when the connect/disconnect button is clicked
        if self.Imuserial_port and self.Imuserial_port.isOpen():
            # If port is open, disconnect it
            # self.timer.stop()  # Start the timer
            self.Imuserial_port.close()
            self.IMUconnectButton.setText('IMU_Connect')
            self.IMUconnectButton.setStyleSheet("background-color: red; color: black;")
        else:
             # Open the selected serial port
            port_name = self.comboIMU.currentText().split()[0]  # Get only the port name
            self.Imuserial_port = QSerialPort(port_name)
            self.Imuserial_port.setBaudRate(QSerialPort.Baud115200)
            
            # self.Imuserial_port.setBaudRate(QSerialPort.Baud9600)
            self.Imuserial_port.setDataBits(QSerialPort.Data8)
            self.Imuserial_port.setParity(QSerialPort.NoParity)
            self.Imuserial_port.setStopBits(QSerialPort.OneStop)
            self.Imuserial_port.setFlowControl(QSerialPort.NoFlowControl)

            if self.Imuserial_port.open(QSerialPort.ReadWrite):
                # Start the thread to read data
                # self.ImuRXthread = SerialReaderThread(self.Imuserial_port)
                # self.ImuRXthread.data_received.connect(self.display_data)
                # self.ImuRXthread.start()                
                self.IMUconnectButton.setText('IMU_Disconnect')
                self.IMUconnectButton.setStyleSheet("background-color: green; color: black;")     
                self.IMUtimer.start()
            else:
                self.textEdit.append(f"Failed to open {port_name}")                
                if self.Imuserial_port is None:
                    raise ValueError("Failed to initialize the serial port.")                    
            self.Imuserial_port.errorOccurred.connect(self.handle_error)

    def MOTtoggle_connection(self):        
        if self.motors.client != None and self.motors.client.is_socket_open():
            self.motors.set_rpm(0,0)
            self.MOTconnectButton.setText('MOT_Connect')
            self.MOTconnectButton.setStyleSheet("background-color: red; color: black;")
            self.motors.SerialportClosed()
        else:
            port_name = self.comboMOT.currentText().split()[0]  # Get only the port name
            self.MOTconnectButton.setText('MOT_Disconnect')       
            self.MOTconnectButton.setStyleSheet("background-color: green; color: black;")     
            self.motors.SetSerialport(port_name)
            

    def Traytoggle_connection(self):
        # Called when the connect/disconnect button is clicked
        if self.Trayserial_port and self.Trayserial_port.isOpen():
            # If port is open, disconnect it
            # self.timer.stop()  # Start the timer
            self.Trayserial_port.close() 
            print('Disconnecting to Tray serial')
        else:
             # Open the selected serial port
            port_name = self.comboTRAY.currentText().split()[0]  # Get only the port name
            self.Trayserial_port = QSerialPort(port_name)
            # self.RPlidarserial_port.setBaudRate(QSerialPort.Baud115200)
            self.Trayserial_port.setBaudRate(QSerialPort.Baud115200)
            self.Trayserial_port.setDataBits(QSerialPort.Data8)
            self.Trayserial_port.setParity(QSerialPort.NoParity)
            self.Trayserial_port.setStopBits(QSerialPort.OneStop)
            self.Trayserial_port.setFlowControl(QSerialPort.NoFlowControl)

            if self.Trayserial_port.open(QSerialPort.ReadWrite):
                # Start the thread to read data
                
                print('Connecting to Tray serial')
                
            else:
                self.textEdit.append(f"Tray serial Failed to open {port_name}")                
                if self.Trayserial_port is None:
                    raise ValueError("Tray serial Failed to initialize the serial port.")    
            self.Trayserial_port.errorOccurred.connect(self.handle_error)

    def RPtoggle_connection(self):
        # Called when the connect/disconnect button is clicked
        if self.RPlidarserial_port and self.RPlidarserial_port.isOpen():
            # If port is open, disconnect it
            # self.timer.stop()  # Start the timer
            self.RPlidarserial_port.close()
            self.RPconnectButton.setText('RP_Connect')
            self.RPconnectButton.setStyleSheet("background-color: red; color: black;")
        else:
             # Open the selected serial port
            port_name = self.comboRPLIDAR.currentText().split()[0]  # Get only the port name
            self.RPlidarserial_port = QSerialPort(port_name)
            # self.RPlidarserial_port.setBaudRate(QSerialPort.Baud115200)
            self.RPlidarserial_port.setBaudRate(1024000)
            self.RPlidarserial_port.setDataBits(QSerialPort.Data8)
            self.RPlidarserial_port.setParity(QSerialPort.NoParity)
            self.RPlidarserial_port.setStopBits(QSerialPort.OneStop)
            self.RPlidarserial_port.setFlowControl(QSerialPort.NoFlowControl)

            if self.RPlidarserial_port.open(QSerialPort.ReadWrite):
                # Start the thread to read data
                
                self.RPconnectButton.setText('RP_Disconnect')
                self.RPconnectButton.setStyleSheet("background-color: green; color: black;")
                self.RpLidartimer.start()
            else:
                self.textEdit.append(f"Failed to open {port_name}")                
                if self.RPlidarserial_port is None:
                    raise ValueError("Failed to initialize the serial port.")                  
            self.RPlidarserial_port.errorOccurred.connect(self.handle_error)
    def toggle_connection(self):
        
        if self.serial_port and self.serial_port.isOpen():
            # If port is open, disconnect it --------------- Why --- Aleks --> maybe for retrying it
            # self.timer.stop()  # Start the timer
            self.serial_port.close()
            self.connectButton.setText('MCU_Connect')
            self.connectButton.setStyleSheet("background-color: red; color: black;")

            self.sendButton.setEnabled(False)
        else:
            # Open the selected serial port
            port_name = self.comboMCU.currentText().split()[0]  # Get only the port name
            self.serial_port = QSerialPort(port_name)
            self.serial_port.setBaudRate(QSerialPort.Baud115200)
            self.serial_port.setDataBits(QSerialPort.Data8)
            self.serial_port.setParity(QSerialPort.NoParity)
            self.serial_port.setStopBits(QSerialPort.OneStop)
            self.serial_port.setFlowControl(QSerialPort.NoFlowControl)
            # print(port_name) # ----------------Aleks

            if self.serial_port.open(QSerialPort.ReadWrite):
                # Start the thread to read data
                # self.thread = SerialReaderThread(self.serial_port)
                # self.thread.data_received.connect(self.display_data)
                # self.thread.start()
                self.timer.start()  # Start the timer
                self.sendButton.setEnabled(True)
                self.connectButton.setText('MCU_Disconnect')
                self.connectButton.setStyleSheet("background-color: green; color: black;")
            else:
                self.textEdit.append(f"Failed to open {port_name}")
                self.sendButton.setEnabled(False)
                if self.serial_port is None:
                    raise ValueError("Failed to initialize the serial port.")                  
            self.serial_port.errorOccurred.connect(self.handle_error)

    def display_data(self, data):
        # Display the received serial data in hex format
        hex_data = binascii.hexlify(data).decode('utf-8')  # Convert bytes to hex string
        self.textEdit.append(f"Received: {hex_data}")

    def send_serial_data(self):
        global FwDownBin
        self.SendCmd(self)
        
        
        # if self.serial_port and self.serial_port.isOpen():
        #     data = b'Hello Serial Port'  # User-defined data to send
        #     self.serial_port.write(data)  # Send data
        #     self.textEdit.append(f"Sent: {data.decode('utf-8', errors='ignore')}")
        # else:
        #     self.textEdit.append("Serial port is not connected!")

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


    def SetTrayBaseLedCtrl(self,u8Ledflag,u8LedTrayflag):       
        global u8SendPacketIndex  # Access the global index variable
        ucSendDat = bytearray(40)  # Data buffer                           
        # except ValueError as e:
        #     print(f"Invalid input: {e}")
        #     return 0
        # u8Ledflag = int(self.u8LedflagEdit.toPlainText())
        # u8LedTrayflag = int(self.u8LedTrayflagEdit.toPlainText())
        # Ensure values are within the acceptable range
        if not (0 <= u8Ledflag <= 255):
            raise ValueError("u8Ledflag must be between 0 and 255")
        if not (0 <= u8LedTrayflag <= 255):
            raise ValueError("u8LedTrayflag must be between 0 and 255")        
        '''
        # # Populate ucSendDat with data
        ucSendDat[P_STX] = 0X02  # Start byte
        ucSendDat[P_LEN] = 1  # Length
        ucSendDat[P_Idx] = u8SendPacketIndex
        u8SendPacketIndex += 1
        u8SendPacketIndex &= 0xFF
        ucSendDat[P_CMD] = 0xA5
        # Calculate CRC for the data segment
        Datidx = P_Dat 
        crc = crc16(ucSendDat[3:Datidx], ucSendDat[P_LEN])
        ucSendDat[Datidx] = crc & 0xFF  # CRC low byte
        ucSendDat[Datidx + 1] = (crc >> 8) & 0xFF  # CRC high byte
        ucSendDat[Datidx + 2] = ETX  # End byte        
        # print("\n 0xA5")
        '''           
        # # # Populate ucSendDat with data
        ucSendDat[P_STX] = 0X02  # Start byte
        ucSendDat[P_LEN] = 3  # Length
        ucSendDat[P_Idx] = u8SendPacketIndex
        u8SendPacketIndex += 1
        u8SendPacketIndex &= 0xFF
        ucSendDat[P_CMD] = u8CmdSetTrayBaseLedCtrl        
        ucSendDat[P_Dat + 0] = u8Ledflag       
        ucSendDat[P_Dat + 1] = u8LedTrayflag
        # Calculate CRC for the data segment
        Datidx = P_Dat + 2
        crc = crc16(ucSendDat[3:Datidx], ucSendDat[P_LEN])
        ucSendDat[Datidx] = crc & 0xFF  # CRC low byte
        ucSendDat[Datidx + 1] = (crc >> 8) & 0xFF  # CRC high byte
        ucSendDat[Datidx + 2] = ETX  # End byte        
        # Print each byte for debugging
        for i in range(Datidx + 3):
            print(f"{ucSendDat[i]:02x}", end=", ")
        print("SetTrayBaseLedCtrl 0x96")
        if self.serial_port and self.serial_port.isOpen():
            qbyte_send = ucSendDat[:Datidx+3]
            self.serial_port.write(qbyte_send)  # Send data
        else:
            self.textEdit.append("Serial port is not connected!")
    def SetReportSensor(self,ReportCtrl):       
        global u8SendPacketIndex  # Access the global index variable
        ucSendDat = bytearray(40)  # Data buffer                           
        u8Ledflag = int(self.u8LedflagEdit.toPlainText())
        
        # Ensure values are within the acceptable range
        if not (0 <= u8Ledflag <= 255):
            raise ValueError("u8Ledflag must be between 0 and 255")
        
        # Populate ucSendDat with data
        # Header
        ucSendDat[P_STX] = 0X02  # Start byte
        ucSendDat[P_LEN] = 2  # Length
        ucSendDat[P_Idx] = u8SendPacketIndex
        u8SendPacketIndex += 1  
        u8SendPacketIndex &= 0xFF   

        #Data
        ucSendDat[P_CMD] = u8CmdSetReportSensor 
        ucSendDat[P_Dat + 0] = 0    # Watchdog  

        # Calculate CRC for the data segment
        Datidx = P_Dat + 1
        crc = crc16(ucSendDat[3:Datidx], ucSendDat[P_LEN])

        #Tail
        ucSendDat[Datidx] = crc & 0xFF  # CRC low byte
        ucSendDat[Datidx + 1] = (crc >> 8) & 0xFF  # CRC high byte
        ucSendDat[Datidx + 2] = ETX  # End byte   

        # Print each byte for debugging
        for i in range(Datidx + 3):
            print(f"{ucSendDat[i]:02x}", end=", ")
        print("\n u8CmdSetReportSensor")
        if self.serial_port and self.serial_port.isOpen():
            qbyte_send = ucSendDat[:Datidx+3]
            self.serial_port.write(qbyte_send)  # Send data
        else:
            self.textEdit.append("Serial port is not connected!")
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


    def Get_TraySensorDat(self):       
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
        ucSendDat[P_CMD] = GetTRAYSensorDATA        
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
        # try:
        #     self.u8Ledflag = int(self.u8LedflagEdit.toPlainText())
        #     self.u8LedTrayflag = int(self.u8LedTrayflagEdit.toPlainText())
        #     # Ensure values are within the acceptable range
        #     if not (0 <= self.u8Ledflag <= 255):
        #         raise ValueError("u8Ledflag must be between 0 and 255")
        #     if not (0 <= self.u8LedTrayflag <= 255):
        #         raise ValueError("u8LedTrayflag must be between 0 and 255")
        #     print(f"Send serial")
            
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
        # except ValueError as e:
        #     print(f"Invalid input: {e}")
        #     return 0
        
        # Populate ucSendDat with data
        ucSendDat[P_STX] = 0X02  # Start byte
        ucSendDat[P_LEN] = 3   # Length
        ucSendDat[P_Idx] = u8SendPacketIndex
        u8SendPacketIndex += 1
        u8SendPacketIndex &= 0xFF
        ucSendDat[P_CMD] = u8CmdSetReadyToUpdateCtrl
        ucSendDat[P_Dat + 0] = self.combo_box.currentIndex()
        ucSendDat[P_Dat + 1] = 1 # set Mcu Update

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
        else:
            self.textEdit.append("Serial port is not connected!")

    def onShowCtrlBox(self,device):
        self.textEdit.move(10,205) 
        Ypos = 2000
        self.UpdateInfo(Ypos,10)
        self.TOF_Info(Ypos,10)
        self.LED_ctrlPosition(Ypos,10)
        self.ImuPosInfo(Ypos,10)
        self.US_And_SW_Battey_info(Ypos,10)
        self.Mot_Info(Ypos,10)
        self.Lidar_Info(Ypos,10)            
        Ypos = self.DP_POS_Y
        if device == 'LED':        
            Ypos = self.LED_ctrlPosition(Ypos,10)            
            self.textEdit.move(10,Ypos)   
        elif device == 'IMU':
            Ypos = self.ImuPosInfo(Ypos,10)
            self.textEdit.move(10,Ypos)   
        elif device == 'TRAY':
            Ypos = self.TOF_Info(Ypos,10)
            self.textEdit.move(10,Ypos)   
        elif device == 'MCU Download':
            Ypos = self.UpdateInfo(Ypos,10)            
            self.textEdit.move(10,Ypos)   
        elif device == 'SendReport':
            Ypos = self.US_And_SW_Battey_info(Ypos,10)            
            self.textEdit.move(10,Ypos)   
        elif device == 'Mot':
            Ypos = self.Mot_Info(Ypos,10)            
            self.textEdit.move(10,Ypos)   
        elif device == 'Lidar':
            
            
            Ypos = self.Lidar_Info(Ypos,300)            
            self.textEdit.move(10,Ypos)   
    
    def on_CheckEnabled(self):
        self.onShowCtrlBox('default')
        self.IMUcheckBox.setEnabled(True)
        self.DownloadcheckBox.setEnabled(True)
        self.ReportUpdate.setEnabled(True)
        self.LEDctrl.setEnabled(True)   
        self.TrayUpdate.setEnabled(True)
        self.MotorCheckBox.setEnabled(True)
        self.RPLidarCheck.setEnabled(True)
        self.progress_bar.setVisible(False) 
    def on_CheckDisabled(self):
        self.IMUcheckBox.setEnabled(False)
        self.DownloadcheckBox.setEnabled(False)
        self.ReportUpdate.setEnabled(False)
        self.LEDctrl.setEnabled(False)   
        self.TrayUpdate.setEnabled(False)
        self.MotorCheckBox.setEnabled(False)
        self.RPLidarCheck.setEnabled(False)
        self.progress_bar.setVisible(False) 
    def on_Downloadcheckbox_state_changed(self,state):
        print(state)
        if state == 2:  # Checked 상태 (2는 Qt.Checked)            
            print('checked')
            self.onShowCtrlBox('MCU Download')
            self.on_CheckDisabled()
            self.DownloadcheckBox.setEnabled(True)
        elif state == 0:  # Unchecked 상태 (0은 Qt.Unchecked)
            self.on_CheckEnabled()
                        
            print(f"({self.comboMCU.currentText()})" in "(XR21V1414 USB UART Ch A)",f"({self.comboMCU.currentText()})")


    def on_Lidar_state_changed(self,state):
        if state == 2:  # Checked 상태 (2는 Qt.Checked)            
            # self.motors = ZLAC8015D.Controller(port_name)
            self.onShowCtrlBox('Lidar')
            self.on_CheckDisabled()
            self.RPLidarCheck.setEnabled(True)            
        elif state == 0:  # Unchecked 상태 (0은 Qt.Unchecked)            
            self.on_CheckEnabled()
            print('on_Lidar_state_changed Unchecked')

    def on_Mot_state_changed(self,state):
        if state == 2:  # Checked 상태 (2는 Qt.Checked)            
            # self.motors = ZLAC8015D.Controller(port_name)
            if self.motors.client != None and self.motors.client.is_socket_open():
                self.motors.clear_alarm()
                self.motors.disable_motor()
                self.motors.set_accel_time(100,100)
                self.motors.set_decel_time(100,100)
                self.motors.set_mode(3)
                self.motors.enable_motor()
                
                self.cmds = [10, 10]
                self.motors.set_rpm(self.cmds[0],self.cmds[1])
                self.MotorTimer.start()

            self.onShowCtrlBox('Mot')
            self.on_CheckDisabled()
            self.MotorCheckBox.setEnabled(True)

        elif state == 0:  # Unchecked 상태 (0은 Qt.Unchecked)            
            if self.motors.client != None and self.motors.client.is_socket_open():
                self.motors.set_rpm(0,0)
                self.MotorTimer.stop()

            self.onShowCtrlBox('default')
            self.on_CheckEnabled()
            print('on_Mot_state_changed Unchecked')
            
    
    def on_IMU_state_changed(self,state):
        if state == 2:  # Checked 상태 (2는 Qt.Checked)            
            
            self.onShowCtrlBox('IMU')
            self.on_CheckDisabled()
            self.IMUcheckBox.setEnabled(True)

        elif state == 0:  # Unchecked 상태 (0은 Qt.Unchecked)            
            self.on_CheckEnabled()
            print('on_LEDctrl_state_changed Unchecked')
            # label.setText("Checkbox is Unchecked")
    def on_LEDctrl_state_changed(self,state):
        if state == 2:  # Checked 상태 (2는 Qt.Checked)            
            print('on_LEDctrl_state_changed checked')
            self.onShowCtrlBox('LED')
            self.on_CheckDisabled()
            self.LEDctrl.setEnabled(True)

        elif state == 0:  # Unchecked 상태 (0은 Qt.Unchecked)            
            self.on_CheckEnabled()
            print('on_LEDctrl_state_changed Unchecked')
            # label.setText("Checkbox is Unchecked")
    def on_TrayUpdate_state_changed(self,state):
        if state == 2:  # Checked 상태 (2는 Qt.Checked)            
            print('on_TrayUpdate_state_changed checked')
            self.Get_TraySensorDat()
            self.onShowCtrlBox('TRAY')
            self.on_CheckDisabled()
            self.TrayUpdate.setEnabled(True)      
        elif state == 0:  # Unchecked 상태 (0은 Qt.Unchecked)                        
            self.onShowCtrlBox('default')
            print('on_TrayUpdate_state_changed Unchecked')
            self.on_CheckEnabled()
    def on_ReportUpdate_state_changed(self,state):
        if state == 2:  # Checked 상태 (2는 Qt.Checked)            
            print('on_ReportUpdate_state_changed checked')
            self.SetReportSensor(1)            
            self.onShowCtrlBox('SendReport')          
            self.on_CheckDisabled()
            self.ReportUpdate.setEnabled(True) 

        elif state == 0:  # Unchecked 상태 (0은 Qt.Unchecked)
            print('on_ReportUpdate_state_changed Unchecked')
            self.SetReportSensor(0)         
            self.on_CheckEnabled()

    def read_selected_index(self, button):
        selected_id = self.Base_fb_button_group.checkedId()  # 클릭된 버튼의 ID 가져오기
        selected_uv_id = self.uv_fb_button_group.checkedId() # 클릭된 버튼의 ID 가져오기
        selected_t1_id = self.tray1_button_group.checkedId()  # 클릭된 버튼의 ID 가져오기
        selected_t2_id = self.tray2_button_group.checkedId()  # 클릭된 버튼의 ID 가져오기
        selected_t3_id = self.tray3_button_group.checkedId()  # 클릭된 버튼의 ID 가져오기
        selected_t4_id = self.tray4_button_group.checkedId()  # 클릭된 버튼의 ID 가져오기
        
        print(f"Selected Index: {selected_id}")  # 라벨 업데이트
        print(f"Selected Index uv: {selected_uv_id}")  # 라벨 업데이트
        print(f"Selected Index t1: {selected_t1_id}")  # 라벨 업데이트
        print(f"Selected Index t2: {selected_t2_id}")  # 라벨 업데이트
        print(f"Selected Index t3: {selected_t3_id}")  # 라벨 업데이트
        print(f"Selected Index t4: {selected_t4_id}")  # 라벨 업데이트

        print(f"Led base+uv+t4: {selected_t4_id+selected_uv_id+selected_id}")  # 라벨 업데이트
        print(f"Led t1,t2,+t3: {selected_t1_id+selected_t2_id+selected_t3_id}")  # 라벨 업데이트
        self.SetTrayBaseLedCtrl(int(selected_t4_id+selected_uv_id+selected_id),int(selected_t1_id+selected_t2_id+selected_t3_id))                        

    def RpLidarSendCmdWithOutPayload(self,Cmd):
        global u8RpLidarSendCmd  # Access the global index variable
        ucSendDat = bytearray(40)  # Data buffer                           
        # Start Flag(1byte)Command(1byte)PayloadSize(1byte)Payload Data(0~255bytes)Checksum(1byte)

        # Populate ucSendDat with data
        ucSendDat[0] = SL_START_FLAG   # start flag
        ucSendDat[1] = Cmd  # Length
        u8RpLidarSendCmd = Cmd
        ucSendDat[2] = 0x00
        # Print each byte for debugging
        # for i in range(3):
            # print(f"{ucSendDat[i]:02x}", end=", ")        
        if self.RPlidarserial_port and self.RPlidarserial_port.isOpen():
            qbyte_send = ucSendDat[:3]
            self.RPlidarserial_port.write(qbyte_send)  # Send data
            self.RPlidarserial_port.flush()  
            print(f"{qbyte_send}", end=", ")
        else:
            self.textEdit.append("RPlidarserial_port is not connected!")

        print(f'RpLidarCmd : {Cmd}')
        # self.serial_port.write(bytes([Cmd]))
        # self.serial_port.flush()

    def SendRplidarCommand(self,index):
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
        if index == 0:
            print('SL_LIDAR_CMD_SCAN')
            self.RpLidarSendCmdWithOutPayload(SL_LIDAR_CMD_SCAN)
        elif index == 1:
            print('SL_LIDAR_CMD_STOP')
            self.RpLidarSendCmdWithOutPayload(SL_LIDAR_CMD_STOP)
        
            # self.RPlidarserial_port            
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

