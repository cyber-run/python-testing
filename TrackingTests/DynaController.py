from dynamixel_sdk import *  # Uses Dynamixel SDK library
import time

class DynaController:
    def __init__(self):
        # Control table address for X-series
        self.ADDR_X_OPERATING_MODE = 11
        self.ADDR_X_VELOCITY_LIMIT = 44
        self.ADDR_X_TORQUE_ENABLE = 64
        self.ADDR_X_GOAL_VELOCITY = 104 
        self.ADDR_X_GOAL_POSITION = 116
        self.ADDR_X_PRESENT_VELOCITY = 128
        self.ADDR_X_PRESENT_POSITION = 132

        # Protocol version
        self.PROTOCOL_VERSION = 2.0  # X-series uses protocol version 2.0

        # Default setting
        self.DXL_ID = 1  # Dynamixel ID: 1
        self.BAUDRATE = 57600  # Dynamixel default baudrate: 57600
        self.DEVICENAME = '/dev/tty.usbserial-FT89FAA7'  # Port name

        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)


    def open_port(self):
        try:
            if self.portHandler.openPort():
                print("Succeeded to open the port")
            else:
                print("Failed to open the port")
                return False

            if self.portHandler.setBaudRate(self.BAUDRATE):
                print("Succeeded to change the baudrate")
            else:
                print("Failed to change the baudrate")
                return False
            return True
        except Exception as e:
            print(f"Error opening port: {e}")
            return False


    def write1ByteData(self, address, value):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, address, value)
        if dxl_comm_result != COMM_SUCCESS:
            print(self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print(self.packetHandler.getRxPacketError(dxl_error))


    def write4ByteData(self, address, value):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID, address, value)
        if dxl_comm_result != COMM_SUCCESS:
            print(self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print(self.packetHandler.getRxPacketError(dxl_error))


    def read1ByteData(self, address):
        dxl_data, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL_ID, address)
        if dxl_comm_result != COMM_SUCCESS:
            print(self.packetHandler.getTxRxResult(dxl_comm_result))
            return None
        elif dxl_error != 0:
            print(self.packetHandler.getRxPacketError(dxl_error))
            return None
        else:
            return dxl_data


    def read4ByteData(self, address):
        dxl_data, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, address)
        if dxl_comm_result != COMM_SUCCESS:
            print(self.packetHandler.getTxRxResult(dxl_comm_result))
            return None
        elif dxl_error != 0:
            print(self.packetHandler.getRxPacketError(dxl_error))
            return None
        else:
            return dxl_data


    @staticmethod
    def to_signed32(n):
        n = n & 0xffffffff
        return (n ^ 0x80000000) - 0x80000000


    def close_port(self):
        self.portHandler.closePort()


if __name__ == '__main__':
    dyna = DynaController()

    if dyna.open_port():
        dyna.write1ByteData(dyna.ADDR_X_TORQUE_ENABLE, 0)

        velocity_limit = dyna.read4ByteData(dyna.ADDR_X_VELOCITY_LIMIT)
        print(f"Velocity limit: {velocity_limit}\n")

        dyna.write1ByteData(dyna.ADDR_X_OPERATING_MODE, 1)
        operating_mode = dyna.read1ByteData(dyna.ADDR_X_OPERATING_MODE)
        print(f"Operating mode: {operating_mode}\n")

        dyna.write4ByteData(dyna.ADDR_X_GOAL_VELOCITY, 0)
        dyna.write1ByteData(dyna.ADDR_X_TORQUE_ENABLE, 1)

        start_pos = dyna.read4ByteData(dyna.ADDR_X_PRESENT_POSITION)

        try:
            vel = 100
            while True:
                vel = -vel
                dyna.write4ByteData(dyna.ADDR_X_GOAL_VELOCITY, vel)
                time.sleep(4)

                pres_pos = start_pos - dyna.read4ByteData(dyna.ADDR_X_PRESENT_POSITION)
                if pres_pos is not None:
                    pres_angle = round(360 * (pres_pos / 4096), 3)
                    print(f"Current position: {pres_angle} degrees")

                pres_vel = dyna.read4ByteData(dyna.ADDR_X_PRESENT_VELOCITY)
                if pres_vel is not None:
                    pres_vel = DynaController.to_signed32(pres_vel)
                    print(f"Current velocity: {pres_vel}\n")

        except KeyboardInterrupt:
            print("Interrupted by user")
            dyna.write1ByteData(dyna.ADDR_X_TORQUE_ENABLE, 0)
            dyna.close_port()

        except Exception as e:
            print(f"Error during operation: {e}")
            dyna.write1ByteData(dyna.ADDR_X_TORQUE_ENABLE, 0)
            dyna.close_port()

