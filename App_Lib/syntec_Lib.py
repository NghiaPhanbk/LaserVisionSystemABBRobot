from pyModbusTCP.client import ModbusClient
import time
from GlobalVariables import *
class robot_syntec():
    def __init__(self):
        self.ip_address = '192.168.0.6'
        self.port = 502
        self.unit_id = 6
        self.timeout = 1000
    def convert_to_32bit(self, reg_high, reg_low):
        # Check if the sign bit is set
        if reg_high & 0x8000:
            # If sign bit is set, perform two's complement conversion
            reg_high = reg_high - 0x10000
        # Combine the high and low registers into a 32-bit value
        decimal_32bit = (reg_high << 16) | reg_low
        return decimal_32bit
    def convert_to_16bit(self, decimal_32bit):
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

    def OFFSET_Z(self, lst):
        for item in lst:
            item[2] =205
        return lst
    def RposC(self):
        # Define the register address to read from
        register_address = 1442
        # Create a Modbus TCP client instance
        client = ModbusClient(self.ip_address, self.port, self.unit_id, self.timeout)
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
            X = self.convert_to_32bit(result[0], result[1])
            Y = self.convert_to_32bit(result[2], result[3])
            Z = self.convert_to_32bit(result[4], result[5])
            Rx = self.convert_to_32bit(result[6], result[7])
            Ry = self.convert_to_32bit(result[8], result[9])
            Rz = self.convert_to_32bit(result[10], result[11])
        else:
            # Failed to read the register
            print("Failed to read register")
        # Disconnect from the PLC
        point0 = [X,Y,Z,Rx,Ry,Rz]
        point = [float(value)/1000 for value in point0]
        print(point)
        client.close()
        return point
    def import_multiple_point(self, trajectory_path):
        regs_addr = 40202
        # Create a Modbus TCP client instance
        client = ModbusClient(self.ip_address, self.port, self.unit_id, self.timeout)
        client.open()
        # Connect to the PLC
        if client.is_open:
            print("Connected to PLC")
        else:
            print("Failed to connect to PLC")
            exit(1)
        import pandas as pd
        data = pd.read_csv(trajectory_path)
        with open(trajectory_path, 'r') as f:
            positions = [[(float(num)) for num in line.split('\t')] for line in f]
            print(positions)
        # data = pd.read_csv(welding_trajectory, sep='\t')
        # positions = data.to_numpy()
        # print(len(positions))
        positions = self.OFFSET_Z(positions)
        print(positions)
        check = client.write_single_register(40201, len(positions)) #number of point write to register 20000
        ori = [-180, 0, 0]
        for i in positions:
            point = i + ori
            point = [int(value*1000) for value in point]
            print(point)
            regs_value = []
            for j in point:
                regs_value = regs_value + list(self.convert_to_16bit(j))
            print(regs_value)
            check = client.write_multiple_registers(regs_addr, regs_value)
            regs_addr = regs_addr +20
            print(check)
        # Check if the read operation was successful
        if check:
            print("OK")
        else:
            # Failed to read the register
            print("Failed to write register")
        # Disconnect from the PLC
        client.close()
    def Start_scanning(self):
        # Define the register address to read from
        register_address = 4003
        # Create a Modbus TCP client instance
        client = ModbusClient(self.ip_address, self.port, self.unit_id, self.timeout)
        client.open()
        # Connect to the PLC
        if client.is_open:
            print("Connected to PLC")
        else:
            print("Failed to connect to PLC")
            exit(1)
        check = client.write_single_register(register_address, 1)
        if check:
            print("OK")
        else:
            # Failed to read the register
            print("Failed to write register")
        # Disconnect from the PLC
        client.close()
    def Start_welding(self):
        # Define the register address to read from
        register_address = 4007
        # Create a Modbus TCP client instance
        client = ModbusClient(self.ip_address, self.port, self.unit_id, self.timeout)
        client.open()
        # Connect to the PLC
        if client.is_open:
            print("Connected to PLC")
        else:
            print("Failed to connect to PLC")
            exit(1)
        check = client.write_single_register(register_address, 1)
        if check:
            print("OK")
        else:
            # Failed to read the register
            print("Failed to write register")
        # Disconnect from the PLC
        client.close()
    def Move_to_calibrate(self):
        # Define the register address to read from
        register_address = 4015
        # Create a Modbus TCP client instance
        client = ModbusClient(self.ip_address, self.port, self.unit_id, self.timeout)
        client.open()
        # Connect to the PLC
        if client.is_open:
            print("Connected to PLC")
        else:
            print("Failed to connect to PLC")
            exit(1)
        check = client.write_single_register(register_address, 1)
        if check:
            print("OK")
        else:
            # Failed to read the register
            print("Failed to write register")
        # Disconnect from the PLC
        client.close()
    def Start_program(self):
        # Define the register address to read from
        register_address = 4011
        # Create a Modbus TCP client instance
        client = ModbusClient(self.ip_address, self.port, self.unit_id, self.timeout)
        client.open()
        # Connect to the PLC
        if client.is_open:
            print("Connected to PLC")
        else:
            print("Failed to connect to PLC")
            exit(1)
        check = client.write_single_register(register_address, 1)
        if check:
            print("OK")
        else:
            # Failed to read the register
            print("Failed to write register")
        # Disconnect from the PLC
        client.close()
    def Move_to_calibrate(self):
        # Define the register address to read from
        register_address = 4003
        # Create a Modbus TCP client instance
        client = ModbusClient(self.ip_address, self.port, self.unit_id, self.timeout)
        client.open()
        # Connect to the PLC
        if client.is_open:
            print("Connected to PLC")
        else:
            print("Failed to connect to PLC")
            exit(1)
        check = client.write_single_register(register_address, 1)
        if check:
            print("OK")
        else:
            # Failed to read the register
            print("Failed to write register")
        # Disconnect from the PLC
        client.close()
    def StartRequest(self):
        # Define the register address to read from
        register_address = 1442
        # Create a Modbus TCP client instance
        client = ModbusClient(self.ip_address, self.port, self.unit_id, self.timeout)
        client.open()
        # Connect to the PLC
        if client.is_open:
            print("Connected to PLC")
        else:
            print("Failed to connect to PLC")
            # exit(1)
            return 0
        # Read the register from the PLC
        result = client.read_holding_registers(register_address, 12)
        print(result)
        # Check if the read operation was successful
        if result:
            print("OKE")
        else:
            # Failed to read the register
            print("Failed to read register")
            return 0
        # Disconnect from the PLC
        client.close()
        return 1
    def Move_to_calibrate(self):
        # Define the register address to read from
        register_address = 4003
        # Create a Modbus TCP client instance
        client = ModbusClient(self.ip_address, self.port, self.unit_id, self.timeout)
        client.open()
        # Connect to the PLC
        if client.is_open:
            print("Connected to PLC")
        else:
            print("Failed to connect to PLC")
            exit(1)
        check = client.write_single_register(register_address, 1)
        if check:
            print("OK")
        else:
            # Failed to read the register
            print("Failed to write register")
        # Disconnect from the PLC
        client.close()
    def stop_scanning(self):
        # Define the register address to read from
        register_address = 4005
        # Create a Modbus TCP client instance
        client = ModbusClient(self.ip_address, self.port, self.unit_id, self.timeout)
        client.open()
        # Connect to the PLC
        if client.is_open:
            print("Connected to PLC")
        else:
            print("Failed to connect to PLC")
            exit(1)
        # Read the register from the PLC
        result = client.read_holding_registers(register_address, 1)
        # print(result)
        # Check if the read operation was successful
        if result:
            # Extract the value from the result
            value = result[0]
            # Print the value
            print("Register value:", value)
        else:
            # Failed to read the register
            print("Failed to read register")
        # Disconnect from the PLC
        client.close()
        return int(value)
    def run_scan_flag(self):
        # Define the register address to read from
        register_address = 4001
        # Create a Modbus TCP client instance
        client = ModbusClient(self.ip_address, self.port, self.unit_id, self.timeout)
        client.open()
        # Connect to the PLC
        if client.is_open:
            # print("Connected to PLC")
            pass
        else:
            # print("Failed to connect to PLC")
            # exit(1)
            return 0
        # Read the register from the PLC
        result = client.read_holding_registers(register_address, 1)
        # print(result)
        # Check if the read operation was successful
        if result:
            print("OKE")
        else:
            # Failed to read the register
            print("Failed to read register")
            return 0
        # Disconnect from the PLC
        client.close()
        return result
if __name__ == "__main__":
    add_rgs = 40002
    # robot_syntec.Start_scanning()
    # Start_welding()
    robot = robot_syntec()
    # x = robot.Start_program()
    robot.import_multiple_point(welding_trajectory)
    # print(x)