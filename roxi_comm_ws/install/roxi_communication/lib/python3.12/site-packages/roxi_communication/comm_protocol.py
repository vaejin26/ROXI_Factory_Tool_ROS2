import serial

P_STX = 0
P_LEN = 1
P_Idx = 2
P_CMD = 3
P_Dat = 4
P_ETX = 7
P_SEND = 8

ETX = 0x03

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
        print("here")

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