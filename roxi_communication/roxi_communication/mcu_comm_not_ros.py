import serial
import serial.tools.list_ports
from time import sleep

# ------------------ 통신 상수 ------------------
P_STX = 0
P_LEN = 1
P_Idx = 2
P_CMD = 3
P_Dat = 4
ETX = 0x03
GetMcuVersion = 0xA4

# ------------------ 상태 정의 ------------------
IDLE_S = 0
LEN_S = 1
PIDX_S = 2
DATA_S = 3
CRC_S = 4
ETX_S = 5

state = IDLE_S
uart_buf = bytearray(256)
cnt = 0

# ------------------ CRC 계산 ------------------
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

# ------------------ 포트 찾기 ------------------
def find_mcu_port():
    expected_port = "/dev/ttyUSB0"
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if expected_port in port.device:
            print(f"✅ MCU 연결됨: {port.device}")
            return port.device
    print("❌ MCU 포트를 찾을 수 없습니다.")
    return None

# ------------------ MCU 버전 요청 ------------------
def send_mcu_version_request(ser):
    index = 0x01
    buf = bytearray(10)
    buf[P_STX] = 0x02
    buf[P_LEN] = 1
    buf[P_Idx] = index
    buf[P_CMD] = GetMcuVersion
    crc = crc16(buf[3:4], buf[P_LEN])
    buf[4] = crc & 0xFF
    buf[5] = (crc >> 8) & 0xFF
    buf[6] = ETX
    ser.write(buf[:7])

# ------------------ 응답 파싱 ------------------
def parse_mcu_packet(buf):
    def u16(lo, hi): return (buf[hi] << 8) | buf[lo]

    print("📥 MCU Version Info:")
    for i in range(5):  # Battery + Tray1~4
        base = P_Dat + i * 8
        year  = u16(base + 0, base + 1)
        month = buf[base + 2]
        day   = buf[base + 3]
        major = u16(base + 4, base + 5)
        minor = u16(base + 6, base + 7)

        label = "Battery" if i == 0 else f"Tray{i}"
        if year == 0 and month == 0 and day == 0 and major == 0 and minor == 0:
            print(f"  - {label}: ❌ Not Connected or Invalid")
        else:
            print(f"  - {label}: V{major}.{minor} ({year}.{month}.{day})")

# ------------------ 상태 기반 수신 FSM ------------------
def receive_packet_fsm(ser):
    state, uart_buf, cnt
    while True:
        cc = ser.read(1)
        if not cc:
            continue
        cc = cc[0]

        if state == IDLE_S:
            if cc == 0x02:
                cnt = 0
                uart_buf[cnt] = cc
                state = LEN_S
        elif state == LEN_S:
            uart_buf[cnt + 1] = cc
            state = PIDX_S
        elif state == PIDX_S:
            uart_buf[cnt + 2] = cc
            state = DATA_S
        elif state == DATA_S:
            uart_buf[cnt + 3] = cc
            expected_len = uart_buf[P_LEN]
            data_end = 4 + expected_len  # after CMD
            while cnt + 4 < data_end:
                byte = ser.read(1)
                if not byte:
                    continue
                uart_buf[cnt + 4] = byte[0]
                cnt += 1
            state = CRC_S
        elif state == CRC_S:
            crc_low = ser.read(1)
            crc_high = ser.read(1)
            if not crc_low or not crc_high:
                state = IDLE_S
                continue
            uart_buf[data_end] = crc_low[0]
            uart_buf[data_end + 1] = crc_high[0]
            state = ETX_S
        elif state == ETX_S:
            etx = ser.read(1)
            if not etx or etx[0] != ETX:
                print("❌ ETX 오류 또는 누락")
                state = IDLE_S
                continue
            uart_buf[data_end + 2] = etx[0]
            # CRC 검증
            recv_crc = (uart_buf[data_end + 1] << 8) | uart_buf[data_end]
            calc_crc = crc16(uart_buf[3:data_end], uart_buf[P_LEN])
            if recv_crc == calc_crc:
                parse_mcu_packet(uart_buf)
            else:
                print("❌ CRC 오류, 무시됨")
            print("--------------------------------------------------")
            return

# ------------------ 메인 ------------------
def main():
    mcu_port = find_mcu_port()
    if not mcu_port:
        return

    try:
        ser = serial.Serial(mcu_port, baudrate=115200, timeout=0.1)
        print("📤 MCU 상태 FSM 기반 수신 시작 (중지: Ctrl+C)")

        while True:
            send_mcu_version_request(ser)
            sleep(0.3)
            receive_packet_fsm(ser)
            sleep(1.0)

    except Exception as e:
        print(f"❌ MCU 통신 실패: {e}")

if __name__ == "__main__":
    main()
