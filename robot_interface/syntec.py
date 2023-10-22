from pyModbusTCP.client import ModbusClient
import time, sys
def convert_to_32bit(reg_high, reg_low):
    # Check if the sign bit is set
    if reg_high & 0x8000:
        # If sign bit is set, perform two's complement conversion
        reg_high = reg_high - 0x10000
    # Combine the high and low registers into a 32-bit value
    decimal_32bit = (reg_high << 16) | reg_low
    return decimal_32bit
def convert_to_16bit(decimal_32bit):
    # Kiểm tra dấu âm
    if decimal_32bit < 0:
        # Tính giá trị đối
        abs_value = abs(decimal_32bit)
        # Chuyển đổi thành giá trị bù hai
        complement_value = (1 << 32) - abs_value
        # Lấy giá trị từ thanh ghi cao (16 bit)
        reg_high = (complement_value >> 16) & 0xFFFF
        # Lấy giá trị từ thanh ghi thấp (16 bit)
        reg_low = complement_value & 0xFFFF
        return reg_high, reg_low
    else:
        # Lấy giá trị từ thanh ghi cao (16 bit)
        reg_high = (decimal_32bit >> 16) & 0xFFFF
        # Lấy giá trị từ thanh ghi thấp (16 bit)
        reg_low = decimal_32bit & 0xFFFF
        return reg_high, reg_low
def read_client():
    # Define the PLC's IP address and Modbus TCP port
    plc_ip = '192.168.0.10'
    plc_port = 502
    # Define the register address to read from
    register_address = 1442
    # Create a Modbus TCP client instance
    client = ModbusClient(plc_ip, plc_port, unit_id=10, timeout=1000)
    client.open()
    # Connect to the PLC
    if client.is_open:
        print("Connected to PLC")
    else:
        print("Failed to connect to PLC")
        exit(1)
    # Read the register from the PLC
    result = client.read_holding_registers(register_address, 12)
    print(result)
    # Check if the read operation was successful
    if result:
        # Extract the value from the result
        value = result[0]
        # Print the value
        print("Register value:", value)
        X = convert_to_32bit(result[0], result[1])
        Y = convert_to_32bit(result[2], result[3])
        Z = convert_to_32bit(result[4], result[5])
        Rx = convert_to_32bit(result[6], result[7])
        Ry = convert_to_32bit(result[8], result[9])
        Rz = convert_to_32bit(result[10], result[11])
    else:
        # Failed to read the register
        print("Failed to read register")
    # Disconnect from the PLC
    point0 = [X,Y,Z,Rx,Ry,Rz]
    print(point0)
    client.close()
def write_client():
    # Define the PLC's IP address and Modbus TCP port
    plc_ip = '192.168.0.10'
    plc_port = 502
    # Define the register address to read from
    register_address = 0
    # Create a Modbus TCP client instance
    client = ModbusClient(plc_ip, plc_port, unit_id=10, timeout=1000)
    client.open()
    # Connect to the PLC
    if client.is_open:
        print("Connected to PLC")
    else:
        print("Failed to connect to PLC")
        exit(1)
    # abc = client.write_multiple_registers(60000, 4)
    # Read the register from the PLC
    # result = client.read_holding_registers(1442, 12)
    regs_addr = 40002
    point1 = [480389, -1761, 848384, -173931, 50515, -175889,0,100000,20000]
    point2 = [480389, 50000, 848384, -173931, 50515, -175889,0,100000,20000]
    point3 = [422389, 50000, 848384, -173931, 50515, -175889,0,100000,20000]
    point4 = [422389, -1761, 848384, -173931, 50515, -175889,0,100000,20000]
    regs_value1 = []
    regs_value2 = []
    regs_value3 = []
    regs_value4 = []
    # list =[]
    # for i in range(1,60):
    #     list[i] = 0
    # abc = abc = client.write_multiple_registers(regs_addr, list)
    for j in point1:
        regs_value1 = regs_value1 + list(convert_to_16bit(j))
    for j in point2:
        regs_value2 = regs_value2 + list(convert_to_16bit(j))
    for j in point3:
        regs_value3 = regs_value3 + list(convert_to_16bit(j))
    for j in point4:
        regs_value4 = regs_value4 + list(convert_to_16bit(j))
    abc = client.write_multiple_registers(regs_addr, regs_value1)
    abc = client.write_multiple_registers(regs_addr + 20, regs_value2)
    abc = client.write_multiple_registers(regs_addr + 40, regs_value3)
    abc = client.write_multiple_registers(regs_addr + 60, regs_value4)
    ab = [4]
    abc = client.write_multiple_registers(40001, ab)
    print(abc)
    # Check if the read operation was successful
    if abc:
        print("OK")
    else:
        # Failed to read the register
        print("Failed to write register")
    # Disconnect from the PLC
    client.close()
def write_client1():
    # Define the PLC's IP address and Modbus TCP port
    plc_ip = '192.168.0.10'
    plc_port = 502
    # Define the register address to read from
    register_address = 0
    # Create a Modbus TCP client instance
    client = ModbusClient(plc_ip, plc_port, unit_id=10, timeout=1000)
    client.open()
    # Connect to the PLC
    if client.is_open:
        print("Connected to PLC")
    else:
        print("Failed to connect to PLC")
        exit(1)
    ab = [1]
    abc = client.write_multiple_registers(4003, ab)
    if abc:
        print("OK")
    else:
        # Failed to read the register
        print("Failed to write register")
    # Disconnect from the PLC
    client.close()
def test():
    plc_ip = '192.168.0.10'
    plc_port = 502
    # Define the register address to read from
    register_address = 1442
    # Create a Modbus TCP client instance
    client = ModbusClient(plc_ip, plc_port, unit_id=10, timeout=1000)
    client.open()
    # Connect to the PLC
    # if client.is_open:
    #     print("Connected to PLC")
    # else:
    #     print("Failed to connect to PLC")
    #     exit(1)
    # Read the register from the PLC
    result = client.read_holding_registers(register_address, 12)
    # print(result)
    # Check if the read operation was successful
    if result:
        # Extract the value from the result
        value = result[0]
        # Print the value
        # print("Register value:", value)
        X = convert_to_32bit(result[0], result[1])
        Y = convert_to_32bit(result[2], result[3])
        Z = convert_to_32bit(result[4], result[5])
        Rx = convert_to_32bit(result[6], result[7])
        Ry = convert_to_32bit(result[8], result[9])
        Rz = convert_to_32bit(result[10], result[11])
    else:
        # Failed to read the register
        print("Failed to read register")
    # Disconnect from the PLC
    point0 = [X,Y,Z,Rx,Ry,Rz]
    print(point0)
    client.close()
    return point0
# read_client()
# for i in range(10):
#     x = test()
#     time.sleep(0.02)
#     print(x)
# write_client()
write_client()
for i in range(10):
    x = test()
    time.sleep(0.5)
    print(x)