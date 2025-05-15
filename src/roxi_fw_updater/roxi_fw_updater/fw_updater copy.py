import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_services_default
from std_srvs.srv import Trigger
from std_msgs.msg import String
import serial
import time
import os
import sys
import argparse

# === FSM 및 통신 관련 상수 정의 ===
IDLE_S = 1
LEN_S = 2
PIDX_S = 3
DATA_S = 4
CRC_S = 5
ETX_S = 6

P_STX = 0
P_LEN = 1
P_Idx = 2
P_CMD = 3
P_Dat = 4
P_CRC1 = 5
P_CRC2 = 6
P_ETX = 7

ETX = 0x03
SOH = 0x01
STX = 0x02
EOT = 0x04
ACK = 0x06
NAK = 0x15
CAN = 0x18
CRC = ord('C')

GetMcuVersionInformation = 0xA4
u8CmdSetReadyToUpdateCtrl = 0x97

Xmodemstate = 0
FwDownBin = bytearray()

# === ROS2 노드 정의 ===
class FirmwareUpdaterNode(Node):
    def __init__(self, firmware_path):
        super().__init__('firmware_updater_node')

        # 펌웨어 바이너리 파일 로드
        global FwDownBin
        if not os.path.exists(firmware_path):
            self.get_logger().error(f"[Error] File not found: {firmware_path}")
            sys.exit(1)

        with open(firmware_path, 'rb') as f:
            FwDownBin = f.read()
            self.get_logger().info(f"[Load] Firmware file loaded: {firmware_path} ({len(FwDownBin)} bytes)")

        # 시리얼 포트 설정
        self.serial_port = serial.Serial('/dev/ttyUSB1', baudrate=115200, timeout=0.05)
        self.timer = self.create_timer(0.1, self.on_timer_timeout)

        # 내부 상태
        self.recv_buf = bytearray(256)
        self.uartCnt = 0
        self.state = IDLE_S

        # XMODEM 관련 상태 초기화
        self.XmodemValueInitialize()

        # ROS2 서비스 등록
        self.srv = self.create_service(Trigger, 'start_firmware_update', self.start_update_callback)

        # 상태 Publish를 위한 토픽 생성
        self.status_pub = self.create_publisher(String, 'firmware_status', 10)
        self.publish_status("IDLE")

        self.get_logger().info("[Start] Firmware Updater Node is running... Use 'ros2 service call /start_firmware_update std_srvs/srv/Trigger' to start update.")
        
    def publish_status(self, status_str):
        msg = String()
        msg.data = status_str
        self.status_pub.publish(msg)
        self.get_logger().info(f"[STATUS] {status_str}")

    def XmodemValueInitialize(self):
        global Xmodemstate
        Xmodemstate = 0
        self.sequence = 1
        self.packet_size = 128
        self.error_count = 0
        self.success_count = 0
        self.total_packets = 0
        self.crc_mode = -1
        self.cancel = 0

    def on_timer_timeout(self):
        global Xmodemstate

        if self.serial_port.in_waiting > 0:
            data = self.serial_port.read(self.serial_port.in_waiting)
            for byte in data:
                if Xmodemstate != 0:
                    self.handle_xmodem(byte)
                else:
                    self.uart_process(byte)

    def uart_process(self, cc):
        self.recv_buf[self.uartCnt] = cc
        print(f"cc : {cc}")
        self.uartCnt += 1

        if self.state == IDLE_S:
            if cc == 0x02:
                self.state = LEN_S
                self.uartCnt = 1
        elif self.state == LEN_S:
            if cc > 249 or cc < 1:
                self.state = IDLE_S
                self.uartCnt = 0
            else:
                self.state = PIDX_S
        elif self.state == PIDX_S:
            self.state = DATA_S
        elif self.state == DATA_S:
            if self.uartCnt >= self.recv_buf[P_LEN] + 4:
                self.state = CRC_S
        elif self.state == CRC_S:
            if self.uartCnt >= self.recv_buf[P_LEN] + 5:
                self.state = ETX_S
        elif self.state == ETX_S:
            self.state = IDLE_S
            if cc == ETX:
                crc_recv = (self.recv_buf[self.uartCnt - 2] << 8) | self.recv_buf[self.uartCnt - 3]
                crc_calc = self.crc16(self.recv_buf, self.uartCnt - 3)
                if crc_recv == crc_calc:
                    cmd = self.recv_buf[P_CMD]
                    if cmd == GetMcuVersionInformation:
                        self.get_logger().info("[Recv] MCU Version Info Packet")
                    elif cmd == u8CmdSetReadyToUpdateCtrl:
                        global Xmodemstate
                        Xmodemstate = 1
                        self.get_logger().info("[Recv] MCU Ready to Update")
                        self.publish_status("UPLOADING")
                else:
                    self.get_logger().warn("[CRC Error] Invalid CRC")
                    self.publish_status("ERROR: CRC")

    def get_mcu_version_info(self):
        pkt = bytearray(10)
        pkt[P_STX] = 0x02
        pkt[P_LEN] = 1
        pkt[P_Idx] = 0
        pkt[P_CMD] = GetMcuVersionInformation
        crc = self.crc16(pkt[3:4], 1)
        pkt[5] = crc & 0xFF
        pkt[6] = (crc >> 8) & 0xFF
        pkt[7] = 0x03
        self.serial_port.write(pkt[:8])
        self.get_logger().info("[Send] Get MCU Version Info")

    def start_update_callback(self, request, response):
        self.get_logger().info("[Service] Starting firmware update process...")
        self.send_update_cmd()
        response.success = True
        response.message = "Firmware update initiated."
        self.publish_status("SENT_START_COMMAND")
        return response

    def send_update_cmd(self):
        ucSendDat = bytearray(10)
        ucSendDat[P_STX] = 0x02
        ucSendDat[P_LEN] = 3
        ucSendDat[P_Idx] = 0x01
        ucSendDat[P_CMD] = u8CmdSetReadyToUpdateCtrl
        ucSendDat[P_Dat] = 0x00
        ucSendDat[P_Dat + 1] = 0x01
        Datidx = P_Dat + 2
        crc = self.crc16(ucSendDat[3:Datidx], ucSendDat[P_LEN])
        ucSendDat[Datidx] = crc & 0xFF
        ucSendDat[Datidx + 1] = (crc >> 8) & 0xFF
        ucSendDat[Datidx + 2] = 0x03
        print("ucSednDat :")
        for i in range(Datidx + 3):
            print(f"{ucSendDat[i]:02x}", end=", ")
        self.serial_port.write(ucSendDat[:Datidx + 3])
        self.get_logger().info("[Send] Update command sent to MCU")

    def handle_xmodem(self, cc):
        global Xmodemstate, FwDownBin
        retry = 16

        if Xmodemstate == 1:
            if cc == NAK:
                self.crc_mode = 0
                Xmodemstate = 2
                self.publish_status("XMODEM: Received NAK (Checksum Mode)")
            elif cc == CRC:
                self.crc_mode = 1
                Xmodemstate = 2
                self.publish_status("XMODEM: Received C (CRC Mode)")
            elif cc == CAN:
                self.cancel = 1
                Xmodemstate = 0
                self.publish_status("CANCELED")
            elif cc == EOT:
                Xmodemstate = 0
                self.publish_status("ERROR: Unexpected EOT")
            else:
                self.error_count += 1
                if self.error_count > retry:
                    self.abort()
                    Xmodemstate = 0
                    self.publish_status("ERROR: NAK limit")

        elif Xmodemstate == 3:
            if cc == ACK:
                self.total_packets += 1
                self.success_count += 1
                self.sequence = (self.sequence + 1) % 0x100
                Xmodemstate = 2
                percent = int((self.total_packets * self.packet_size) / len(FwDownBin) * 100)
                self.publish_status(f"PROGRESS: {percent}%")
            else:
                self.error_count += 1
                if self.error_count > retry:
                    Xmodemstate = 0
                    self.publish_status("ERROR: ACK timeout")

        elif Xmodemstate == 4:
            if cc == ACK:
                Xmodemstate = 0
                self.publish_status("SUCCESS")
            else:
                self.serial_port.write(bytes([EOT]))
                self.error_count += 1
                if self.error_count > retry:
                    Xmodemstate = 0
                    self.publish_status("ERROR: EOT failed")

        if Xmodemstate == 2:
            data = bytearray(128)
            endofData = False

            for i in range(self.packet_size):
                idx = self.total_packets * self.packet_size + i
                if idx < len(FwDownBin):
                    data[i] = FwDownBin[idx]
                else:
                    data[i] = 0x1A
                    endofData = True

            if not endofData:
                header = self._make_send_header(self.packet_size, self.sequence)
                checksum = self._make_send_checksum(self.crc_mode, data)
                sendData = header + data + checksum
                self.serial_port.write(sendData)
                Xmodemstate = 3
            else:
                self.XmodemValueInitialize()
                self.serial_port.write(bytes([EOT]))
                self.publish_status("SENT_EOT")

    def abort(self):
        for _ in range(2):
            self.serial_port.write(bytes([CAN]))

    def _make_send_header(self, packet_size, sequence):
        assert packet_size in (128, 1024)
        header = bytearray()
        header.append(SOH if packet_size == 128 else STX)
        header.append(sequence)
        header.append(0xFF - sequence)
        return header

    def _make_send_checksum(self, crc_mode, data):
        if crc_mode:
            crc = self.calc_crc(data)
            return bytearray([crc >> 8, crc & 0xFF])
        else:
            return bytearray([sum(data) % 256])

    def calc_crc(self, data):
        crc = 0
        for b in data:
            crc ^= b << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc <<= 1
        return crc & 0xFFFF

    def crc16(self, pData, length):
        wCrc = 0xffff
        for byte in pData[:length]:
            wCrc ^= byte << 8
            for _ in range(8):
                if wCrc & 0x8000:
                    wCrc = (wCrc << 1) ^ 0x1021
                else:
                    wCrc <<= 1
        return wCrc & 0xffff


def main(args=None):
    parser = argparse.ArgumentParser(description='ROS2 Firmware Updater with XMODEM')
    parser.add_argument('--file', type=str, required=True, help='Path to firmware binary file (.bin)')
    args = parser.parse_args()

    rclpy.init(args=None)
    node = FirmwareUpdaterNode(args.file)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[Exit] Node stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
