import serial
import time
import sys
import platform

SOH = 0x01
STX = 0x02
EOT = 0x04
ACK = 0x06
DLE = 0x10
NAK = 0x15
CAN = 0x18
CRC = 0x43

class FirmwareUpdater:
    def __init__(self, port, firmware_path, logger=None):
        self.port = port
        self.firmware_path = firmware_path
        self.logger = logger or print
        self.serial_port = None
        self.XmodemValueInitialize()

    def XmodemValueInitialize(self):
        self.error_count = 0
        self.quiet = False
        self.cancel = 0
        self.crc_mode = -1
        self.success_count = 0
        self.total_packets = 0
        self.sequence = 1
        self.packet_size = 128
        self.RecivedData = 0

    def update_firmware(self):
        try:
            self.serial_port = serial.Serial(self.port, 115200, timeout=0.1)
        except Exception as e:
            self.log(f"❌ Could not open serial port: {e}")
            return False

        with open(self.firmware_path, 'rb') as f:
            firmware = f.read()

        self.XmodemValueInitialize()
        self.crc_mode = -1
        self.sequence = 1
        self.packet_size = 128
        self.total_packets = 0
        offset = 0
        state = 1  # STATE_WAIT_CRC
        retry = 0
        max_retry = 15

        while True: # timer로 변환, tick 또는 thread
            if state == 1:
                incoming = self.serial_port.read(1)
                print(f"icoming: {incoming}")
                if not incoming:
                    time.sleep(0.05)
                    retry += 1
                    if retry > max_retry:
                        self.log("❌ Timeout waiting for 'C' or 'NAK'")
                        return False
                    continue
                char = incoming[0:1]
                if char == b'C':
                    self.log("✅ CRC mode selected")
                    self.crc_mode = 1
                    state = 2
                    retry = 0
                elif char == bytes([0x15]):  # NAK
                    self.log("✅ Checksum mode selected")
                    self.crc_mode = 0
                    state = 2
                    retry = 0
                elif char == bytes([0x18]):  # CAN
                    self.log("❌ Received CAN. Aborting.")
                    return False

            elif state == 2:
                if offset >= len(firmware):
                    self.serial_port.write(bytes([0x04]))  # EOT
                    self.log("📤 Sent EOT")
                    state = 4
                    continue

                block = bytearray(128)
                for i in range(128):
                    idx = offset + i
                    block[i] = firmware[idx] if idx < len(firmware) else 0x1A

                header = self._make_send_header(128, self.sequence)
                checksum = self._make_send_checksum(self.crc_mode, block)
                packet = header + block + checksum
                self.serial_port.write(packet)
                self.log(f"📤 Sent packet {self.sequence}")
                state = 3

            elif state == 3:
                ack = self.serial_port.read(1)
                if ack and ack[0] == 0x06:
                    percent = int((offset + 128) / len(firmware) * 100)
                    bar = '#' * (percent // 10) + '-' * (10 - (percent // 10))
                    print(f"\\r[{bar}] {percent}% (Packet {self.sequence})", end='', flush=True)
                    self.sequence = (self.sequence + 1) % 256
                    offset += 128
                    self.success_count += 1
                    self.total_packets += 1
                    retry = 0
                    state = 2
                else:
                    retry += 1
                    self.log(f"⚠️ Retry {retry} for packet {self.sequence}")
                    if retry > max_retry:
                        self.log("❌ Too many retries. Aborting.")
                        self.abort()
                        return False
                    state = 2

            elif state == 4:
                ack = self.serial_port.read(1)
                if ack and ack[0] == 0x06:
                    self.log("\\n✅ Firmware update complete.")
                    return True
                else:
                    retry += 1
                    self.log(f"⚠️ Waiting for ACK after EOT (retry {retry})")
                    if retry > max_retry:
                        self.log("❌ No ACK after EOT. Aborting.")
                        return False
                    time.sleep(0.1)

    def _make_send_header(self, packet_size, sequence):
        assert packet_size in (128, 1024), packet_size
        if packet_size == 128:
            header = bytearray([SOH, sequence, 0xFF - sequence])
        else:
            header = bytearray([STX, sequence, 0xFF - sequence])
        return header

    def _make_send_checksum(self, crc_mode, data):
        if crc_mode:
            crc = self.x_calc_check(data)
            return bytearray([crc >> 8, crc & 0xFF])
        else:
            cksum = self.calc_checksum(data)
            return bytearray([cksum])

    def x_calc_check(self, pkt_buf):
        check = 0
        for b in pkt_buf:
            check = self.update_crc(int(b), check)
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

    def abort(self, count=2):
        for _ in range(count):
            self.serial_port.write(CAN)
            time.sleep(0.1)

    def log(self, msg):
        if callable(self.logger):
            self.logger(msg)
        else:
            self.logger.info(msg)