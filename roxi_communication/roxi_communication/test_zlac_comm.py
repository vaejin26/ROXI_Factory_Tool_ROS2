from ZLAC8015D import ZLAC8015D

motor = ZLAC8015D()
motor.SetSerialport("/dev/ttyUSB1")  # 또는 ttyUSB1

try:
    from pymodbus.exceptions import ModbusIOException

    result = motor.client.read_holding_registers(0x200D, 1, unit=motor.ID)
    if result.isError():
        print("❌ Modbus Error:", result)
    else:
        print("✅ Read Success:", result.registers[0])
except ModbusIOException as e:
    print("❗ ModbusIOException:", e)
