import rclpy
from rclpy.node import Node
import argparse
import sys
import os
import serial
import time
import threading
import platform

# === FSM 및 통신 관련 상수 정의 ===
START_S = 0
IDLE_S = 1
LEN_S = 2
PIDX_S = 3
DATA_S = 4
CRC_S = 5
ETX_S = 6
SEND_S = 7

P_STX = 0
P_LEN = 1
P_Idx = 2
P_CMD = 3
P_Dat = 4
P_CRC1 = 5
P_CRC2 = 6
P_ETX = 7
P_SEND = 8

ETX = 0x03
# SOH = 0x01
# STX = 0x02
# EOT = 0x04
# ACK = 0x06
# NAK = 0x15
# CAN = 0x18
# CRC = ord('C')

# ETX = b'\x03'
SOH = b'\x01'
STX = b'\x02'
EOT = b'\x04'
ACK = b'\x06'
DLE = b'\x10'
NAK = b'\x15'
CAN = b'\x18'
CRC = b'C'

u8Uart_TOFDebugPrintflag = 2

GetMcuVersionInformation = 0xA4
u8CmdSetReadyToUpdateCtrl = 0x97

uart5state = IDLE_S
uartTrayCnt = 0
cTofBuff = bytearray(256)

u8SendPacketIndex  = 0
Xmodemstate = 0
FwDownBin = None

class SerailReaderThread(threading.Thread):
    def __init__(self, serial_port, logger=None):
        super().__init__(daemon=True)
        self.serial_port = serial_port
        self.running = True
        self.logger = logger
        self.rx_buffer = bytearray()
        self.lock = threading.Lock()
        self.XmodemValueInitialize()
    
    def run(self):
        global Xmodemstate

        while self.running:
            try:
                if self.serial_port.in_waiting > 0:
                    print("threading")
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    
                    with self.lock:
                        self.rx_buffer.extend(data)
                    if data:
                        for i in range(len(data)):
                            OneByte = int.from_bytes(bytes(data[i:i+1]), byteorder='big')
                            print(f"📥 OneByte: 0x{OneByte:02X} ({chr(OneByte) if 32 <= OneByte <= 126 else '.'})")
                            # print(f"Xmodemstate: {Xmodemstate}")
                            if Xmodemstate != 0:
                                self.handle_xmodem(OneByte)
                                print("handle_xmodem")
                            else:
                                self.uart_process(OneByte)
                                print("uart_process")
                                
                    print("threading step complete")
            except Exception as e:
                if self.logger:
                    self.logger.error(f"Serial read failed in thread: {e}")
                self.running = False
                
            time.sleep(0.01)  # Sleep to prevent high CPU usage
                
    def stop(self):
        self.running = False
        
    def get_buffer(self):
        with self.lock:
            data = bytes(self.rx_buffer)
            self.rx_buffer.clear()
        return data
    
    def uart_process(self, cc):
        global uart5state, uartTrayCnt, cTofBuff, Xmodemstate
        
        # print(f"{cc:02x}",end =' ')
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
            print(f"[FSM] ETX_S 도달. cc: {cc:02X}, uartTrayCnt: {uartTrayCnt}")
            uart5state = IDLE_S
            if cc == ETX:
                print("[FSM] ETX 바이트 일치. CRC 검증 중...")
                crc_dat1 = crc16(cTofBuff, uartTrayCnt - 2)
                crc_dat2 = (cTofBuff[uartTrayCnt - 1] << 8) | cTofBuff[uartTrayCnt - 2]
                if crc_dat1 == crc_dat2:
                    uart5state = IDLE_S
                    print("[FSM] ✅ CRC 통과")
                    if u8CmdSetReadyToUpdateCtrl == cTofBuff[3]:
                        print("✅ ReadyToUpdate 응답 확인 → Xmodemstate 전환")
                        Xmodemstate = 1
                    elif GetMcuVersionInformation == cTofBuff[3]:
                        print("✅ GetMcuVersion 응답 확인")
                        response = cTofBuff[:uartTrayCnt + 1]
                        parse_mcu_version_response(response)
            else:
                print("❌ CRC 오류 발생")

        
    def XmodemValueInitialize(self):
        print("Xmodem Parameter Initialized")
        global Xmodemstate
        Xmodemstate = 0
        self.error_count = 0
        self.quiet = False
        self.cancel = 0
        self.crc_mode = -1
        self.success_count = 0
        self.total_packets = 0
        self.sequence = 1
        self.packet_size = 128
        self.RecivedData = 0

    def handle_xmodem(self, cc):
        global FwDownBin, Xmodemstate
        retry = 16
        char = cc.to_bytes(1, byteorder='big')
        # char = cc
        
        print(f"Xmodemstate: {Xmodemstate}")
        
        # with open(self.firmware_path, 'rb') as f:
        #     firmware = f.read()
        # print("Base File Size", len(firmware))

        if Xmodemstate == 1:  # Wait for 'C' or 'NAK'
            if char:
                if char == NAK:
                    print("✅ standard checksum requested. (NAK)")
                    self.crc_mode = 0
                    Xmodemstate = 2
                    return True
                elif char == CRC:  # 'C'
                    self.log("✅ 16-bit CRC requested.")
                    self.crc_mode = 1
                    Xmodemstate = 2
                    return True
                elif char == CAN:
                    if not self.quiet:
                        print("Received CAN.", file=sys.stderr)
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
                    print(f"send error: expected NAK, CRC, EOT or CAN; got {char!r}")
                
            self.error_count += 1
            if self.error_count > retry:
                print('send error: error_count reached %d, aborting.', retry)
                self.abort(timeout=60)
                Xmodemstate = 0
                self.error_count = 0
                return False

        elif Xmodemstate == 2:  # Send packet
            global FwDownBin
            data = bytearray(128)
            endofData = False
            for i in range(self.packet_size):
                if(len(FwDownBin) > self.total_packets * self.packet_size + i):
                    data[i] = FwDownBin[self.total_packets * self.packet_size + i]
                    print(f"x = {self.total_packets*self.packet_size + i}")
                else:
                    data[i] = 0x1A
                    endofData = True
                    
            if endofData == False:
                header = self._make_send_header(self.packet_size, self.sequence)
                checksum = self._make_send_checksum(self.crc_mode, data)
                packet = header + data + checksum
                self.serial_port.write(packet)
                self.log(f"📤 Sending raw packet:")
                for i in bytearray(packet):
                    print(f"{i:02x}", end=", ")
                print("\n")
                self.log(f"📤 Sent packet {self.sequence}")
                Xmodemstate = 3
            else:
                Xmodemstate = 0
                self.XmodemValueInitialize()
                self.serial_port.write(EOT)
                print('send: at EOF')
                
        elif Xmodemstate == 3:  # Wait for ACK or NAK
            if char == ACK:
                self.total_packets += 1
                self.success_count += 1
                self.error_count = 0
                self.sequence = (self.sequence + 1) % 0x100
                Xmodemstate = 2
            else:
                self.error_count += 1
                Xmodemstate = 3
                if self.error_count > retry:
                    print("too many errors, aborting.")
                    Xmodemstate = 0

        elif Xmodemstate == 4:  # Wait for ACK after EOT
            if char == ACK:
                self.error_count = 0
                Xmodemstate = 0
                print('send EOT Get ACK.')
            else:
                print('send error: expected ACK; got %r', char)
                self.serial_port.write(EOT)
                self.error_count += 1
                if self.error_count > retry:
                    Xmodemstate = 0
                    print('send EOT Get ACK.')

    def _make_send_header(self, packet_size, sequence):
        assert packet_size in (128, 1024), packet_size
        _bytes = []
        if packet_size == 128:
            _bytes.append(ord(SOH))
        elif packet_size == 1024:
            _bytes.append(ord(STX))
        _bytes.extend([sequence, 0xFF - sequence])
        return bytearray(_bytes)

    def _make_send_checksum(self, crc_mode, data):
        _bytes = []
        if crc_mode:
            crc = self.x_calc_check(data)
            _bytes.extend([crc >> 8, crc & 0xFF])
        else:
            crc = self.calc_checksum(data)
            _bytes.append(crc)
            
        return bytearray(_bytes)

    def x_calc_check(self, pkt_buf):
        check = 0
        for i in range(128):
            check = self.update_crc(pkt_buf[i], check)
        return check

    def update_crc(self, b, crc):
        if isinstance(b, bytes):  # ✅ 안전하게 변환
            b = b[0]
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
        return crc & 0xFFFF

    def calc_checksum(self, data, checksum=0):
        if platform.python_version_tuple() >= ('3', '0', '0'):
            return (sum(data) + checksum) % 256
        else:
            return (sum(map(ord, data)) + checksum) % 256

    def abort(self, count=2, timeout=60):
        for _ in range(count):
            self.putc(CAN, timeout)

    def log(self, msg):
        if callable(self.logger):
            self.logger(msg)
        else:
            self.logger.info(msg)

def main(args=None):
    global FwDownBin
    rclpy.init(args=args)
    node = Node("fw_updater")

    parser = argparse.ArgumentParser(description="Firmware Update via XMODEM")
    parser.add_argument('--port', required=True, help='Serial port (e.g., /dev/ttyUSB0)')
    parser.add_argument('--file', required=True, help='Path to FW .bin file')

    argv = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    args = parser.parse_args(argv)
    logger = node.get_logger()

    try:
        ser = serial.Serial(args.port, baudrate=115200, timeout=1)
        logger.info(f"✅ Serial port {args.port} opened successfully.")
    except Exception as e:
        logger.error(f"❌ Failed to open serial port: {e}")
        rclpy.shutdown()
        return

    if not os.path.isfile(args.file):
        logger.error(f"❌ Firmware file does not exist: {args.file}")
        rclpy.shutdown()
        return
    else:
        with open(args.file, 'rb') as f:
            firmware = f.read()
        logger.info(f"✅ Firmware file found: {args.file}")
        print("Base File Size", len(firmware))
        FwDownBin = firmware
        
    reader_thread = SerailReaderThread(ser, logger)
    reader_thread.start()
    
    Get_MCU_Version_Info(ser)
    print("Get mcu version info...")
    time.sleep(0.05)
    
    confirm = input("\nProceed with FW update? [y/N]: ").strip().lower()
    if confirm != 'y':
        logger.info("⚠️ Update canceled by user.")
        reader_thread.stop()
        reader_thread.join()
        rclpy.shutdown()
        ser.close()
        return
    else:
        send_ready_to_update_command(ser)
        
    time.sleep(1)
    while Xmodemstate != 1:
        time.sleep(1)

    
def Get_MCU_Version_Info(serial_port: serial.Serial): # do not modify
        global u8SendPacketIndex
        ucSendDat = bytearray(40)  # Data buffer

        ucSendDat[P_STX] = 0X02  # Start byte
        ucSendDat[P_LEN] = 1   # Length
        ucSendDat[P_Idx] = u8SendPacketIndex
        # u8SendPacketIndex += 1
        # u8SendPacketIndex & 0xFF
        ucSendDat[P_CMD] = GetMcuVersionInformation  # Command

        DatIdx = P_CMD + 1
        crc = crc16(ucSendDat[3:DatIdx], ucSendDat[P_LEN])
        ucSendDat[DatIdx] = crc & 0xFF  # CRC low byte
        ucSendDat[DatIdx + 1] = (crc >> 8) & 0xFF  # CRC high byte
        ucSendDat[DatIdx + 2] = ETX  # End byte

        for i in range(DatIdx + 3):
            print(f"{ucSendDat[i]:02x}", end=", ")
        serial_port.write(ucSendDat[:DatIdx + 3])  # Send data
        
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
                
def send_ready_to_update_command(serial_port: serial.Serial):
        global u8SendPacketIndex
        ucSendDat = bytearray(40)
        ucSendDat[P_STX] = 0x02  # STX
        ucSendDat[P_LEN] = 3  # LEN
        ucSendDat[P_Idx] = 1  # IDX
        u8SendPacketIndex += 1
        u8SendPacketIndex % 0xFF
        
        ucSendDat[P_CMD] = u8CmdSetReadyToUpdateCtrl  # ReadyToUpdate
        ucSendDat[P_Dat] = 0
        ucSendDat[P_Dat + 1] = 1
        
        DatIdx = P_Dat +2
        crc = crc16(ucSendDat[3:DatIdx], ucSendDat[P_LEN])
        ucSendDat[DatIdx] = crc & 0xFF
        ucSendDat[DatIdx + 1] = (crc >> 8) & 0xFF
        ucSendDat[DatIdx + 2] = ETX
        
        for i in range(DatIdx + 3):
            print(f"{ucSendDat[i]:02x}", end=", ")
        serial_port.write(ucSendDat[:DatIdx + 3])  # Send data

        print("📤 Sent ReadyToUpdate (0x97) command to MCU")


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