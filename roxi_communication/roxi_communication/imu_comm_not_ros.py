import serial
import serial.tools.list_ports
import struct
import time

def find_imu_port():
    device_ports = {
        'IMU': '/dev/ttyUSB2',
    }
    
    ports = serial.tools.list_ports.comports()
    available_ports = [port.device for port in ports]
    
    connected_ports = {}
    
    for name, expected_port in device_ports.items():
        matched = next((port for port in available_ports if expected_port in port), None)
        if matched:
            print(f"{name} connected: {matched}")
            connected_ports[name] = matched
        else:
            print(f"Cannot find {name}.")
            
    return connected_ports.get('IMU')

def parse_sensor_packet(raw_bytes, scale):
    x_raw = struct.unpack('<h', raw_bytes[0:2])[0]
    y_raw = struct.unpack('<h', raw_bytes[2:4])[0]
    z_raw = struct.unpack('<h', raw_bytes[4:6])[0]
    x = x_raw / 32768.0 * scale
    y = y_raw / 32768.0 * scale
    z = z_raw / 32768.0 * scale
    return x, y, z

def read_IMU_data(ser):
    while True:
        header = ser.read(1)
        if header != b'\x55':
            continue

        data = ser.read(10)
        if len(data) != 10:
            continue

        packet_type = data[0]

        if packet_type == 0x51:
            ax, ay, az = parse_sensor_packet(data[1:7], 16.0)
            print(f"📦 Accel : ax={ax:.2f}, ay={ay:.2f}, az={az:.2f}")
        elif packet_type == 0x52:
            wx, wy, wz = parse_sensor_packet(data[1:7], 2000.0)
            print(f"📦 Gyro  : wx={wx:.2f}, wy={wy:.2f}, wz={wz:.2f}")
        elif packet_type == 0x53:
            roll, pitch, yaw = parse_sensor_packet(data[1:7], 180.0)
            print(f"📦 Angle : roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}")

def main():
    imu_port = find_imu_port()
    if not imu_port:
        return

    try:
        ser = serial.Serial(imu_port, baudrate=115200, timeout=1)
        print("📤 IMU 데이터 수신 시작")
        read_IMU_data(ser)
    except Exception as e:
        print(f"❌ IMU 연결 실패: {e}")

if __name__ == "__main__":
    main()