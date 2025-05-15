import serial

P_STX = 0
P_LEN = 1
P_Idx = 2
P_CMD = 3
P_Dat = 4
P_ETX = 7
P_SEND = 8

ETX = 0x03


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

GetMcuVersion = 0xA4
GetTraySensorDat = 0xA6

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

def Get_MCU_Version_Info(serial_port: serial.Serial):
    index = 0x01
    ucSendDat = bytearray(40)  # Data buffer

    ucSendDat[P_STX] = 0X02  # Start byte
    ucSendDat[P_LEN] = 1   # Length
    ucSendDat[P_Idx] = index
    ucSendDat[P_CMD] = GetMcuVersion  # Command
    
    dat_idx = P_CMD + 1
    
    crc = crc16(ucSendDat[3:dat_idx], ucSendDat[P_LEN])
    ucSendDat[dat_idx] = crc & 0xFF  # CRC low byte
    ucSendDat[dat_idx + 1] = (crc >> 8) & 0xFF  # CRC high byte
    ucSendDat[dat_idx + 2] = ETX  # End byte
    
    serial_port.write(ucSendDat[:dat_idx + 3])  # Send data
    return True

def parse_mcu_version_response(response: bytes):
    if len(response) < 44:
        print("❌ 응답 길이 부족:", len(response))
        return

    start = 4  # 데이터는 STX(0x02) ~ CMD 이후부터 시작
    def u16(lo, hi): return (response[hi] << 8) | response[lo]

    ver_year = []
    ver_month = []
    ver_day = []
    ver_major = []
    ver_minor = []

    print("📥 MCU Version Info:")

    for i in range(5):  # 1개는 배터리, 4개는 tray
        base = start + i * 8
        year = u16(base + 0, base + 1)
        month = response[base + 2]
        day = response[base + 3]
        major = u16(base + 4, base + 5)
        minor = u16(base + 6, base + 7)

        ver_year.append(year)
        ver_month.append(month)
        ver_day.append(day)
        ver_major.append(major)
        ver_minor.append(minor)

        if year == 0 and month == 0 and day == 0 and major == 0 and minor == 0:
            if i == 0:
                print("  - Battery: ❌ Not Connected or Invalid")
            else:
                print(f"  - Tray{i}: ❌ Not Connected or Invalid")
        else:
            label = "Battery" if i == 0 else f"Tray{i}"
            print(f"  - {label}: V{major}.{minor} ({year}.{month}.{day})")
            
def Get_Tray_Sensor_Data(serial_port: serial.Serial):
    index = 0x01
    ucSendDat = bytearray(10)
    
    ucSendDat[P_STX] = 0x02
    ucSendDat[P_LEN] = 0x01
    ucSendDat[P_Idx] = index
    ucSendDat[P_CMD] = GetTraySensorDat
    
    dat_idx = P_CMD + 1
    crc = crc16(ucSendDat[3:dat_idx], ucSendDat[P_LEN])
    ucSendDat[dat_idx + 0] = crc & 0xFF
    ucSendDat[dat_idx + 1] = (crc >> 8) & 0xFF
    ucSendDat[dat_idx + 2] = ETX
    
    serial_port.write(ucSendDat[:dat_idx + 3])  # Send data
    
def send_rplidar_command(serial_port, cmd):
    serial_port.write(bytearray([SL_START_FLAG, cmd]))