import rclpy
from rclpy.node import Node
import argparse
import sys
import os
import serial
import time
from .xmodem_updater import FirmwareUpdater
from .comm_protocol import Get_MCU_Version_Info, parse_mcu_version_response, crc16

def send_ready_to_update_command(ser):
    cmd = bytearray(40)
    cmd[0] = 0x02  # STX
    cmd[1] = 3  # LEN
    cmd[2] = 0x01  # IDX
    cmd[3] = 0x97  # ReadyToUpdate
    cmd[4] = 0
    cmd[5] = 1
    
    crc = crc16(cmd[3:6], cmd[1])
    cmd[6] = crc & 0xFF
    cmd[7] = (crc >> 8) & 0xFF
    cmd[8] = 0x03  # ETX
    for i in range(9):
        print(f"{cmd[i]:02x}", end=", ")
    print("\n u8CmdSetReadyToUpdateCtrl")
    if ser and ser.is_open:
        ser.write(cmd[:9])
    
    print("📤 Sent ReadyToUpdate (0x97) command to MCU")
    
    # Wait for 'C' (0x43) from bootloader indicating XMODEM start request
    print("⏳ Waiting for 'C' from MCU...")
    start_time = time.time()
    timeout_sec = 10

    while time.time() - start_time < timeout_sec:
        try:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)  # readAll() 대체
                for b in data:
                    val = b if isinstance(b, int) else b[0]  # Python 3-safe
                    print(f"📥 Received: {val:02X} ({chr(val) if 32 <= val <= 126 else '.'})")
                    if val == 0x43:  # 'C'
                        print("✅ Received 'C'. MCU is ready for XMODEM transfer.")
                        return True
        except Exception as e:
            print(f"❌ Serial read failed: {e}")
        time.sleep(0.05)

    print("⚠️ Did not receive 'C' within timeout. MCU might not be in bootloader mode.")

def main(args=None):
    rclpy.init(args=args)
    node = Node("fw_updater")

    parser = argparse.ArgumentParser(description="Firmware Update via XMODEM")
    parser.add_argument('--port', required=True, help='Serial port (e.g., /dev/ttyUSB0)')
    parser.add_argument('--file', required=True, help='Path to firmware .bin file')

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
        logger.info(f"✅ Firmware file found: {args.file}")

    try:
        logger.info(f"📤 Sending MCU version request to {args.port}")
        Get_MCU_Version_Info(ser)
        response = ser.read_until(expected=b'\\x03', size=64)
        logger.info("📥 RAW HEX: " + ' '.join(f"{b:02X}" for b in response))

        if len(response) < 44:
            logger.error("❌ MCU 버전 응답이 충분하지 않음. Aborting.")
            rclpy.shutdown()
            return

        parse_mcu_version_response(response)

        send_ready_to_update_command(ser)
        time.sleep(0.5)
        ser.close()
    except Exception as e:
        logger.error(f"❌ MCU 통신 실패: {e}")
        rclpy.shutdown()
        return

    print()
    confirm = input("Proceed with firmware update? [y/N]: ").strip().lower()
    if confirm != 'y':
        logger.info("⚠️ Update canceled by user.")
        rclpy.shutdown()
        return

    try:
        updater = FirmwareUpdater(args.port, args.file, logger)
        success = updater.update_firmware()
        if success:
            logger.info("✅ Firmware update successful")
        else:
            logger.error("❌ Firmware update failed")
    except Exception as e:
        logger.error(f"Exception during firmware update: {e}")
    finally:
        rclpy.shutdown()