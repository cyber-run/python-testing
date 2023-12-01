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
        self.DEVICENAME = 'COM5'  # Port name

        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

    def open_port(self) -> bool:
        '''
        Open serial port for communication with servo.
        
        Returns:
        - bool: True if port opened successfully, False otherwise.'''

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

    def set_pos(self, pos: float = 180) -> None:
        '''
        Set servo position in degrees.

        Parameters:
        - pos (float): Desired position in degrees.
        '''
        # Convert from degrees to encoder position
        pos = int(pos * 4095 / 360)

        print(f"Setting position to {pos} ticks")

        # Write to servo
        self.write4ByteData(self.ADDR_X_GOAL_POSITION, pos)
    
    def get_pos(self) -> float:
        # Retrieve current encoder position as unsigned 32-bit integer
        pos = self.read4ByteData(self.ADDR_X_PRESENT_POSITION)

        # Convert to signed 32-bit integer
        pos = DynaController.to_signed32(pos)

        # Convert to degrees
        pos = round(360 * (pos / 4095), 3)

        return pos
    
    def set_vel(self, vel: float = 0) -> None:
        '''
        Set servo velocity in degrees per second.

        Parameters:
        - vel (float): Desired velocity in degrees per second.
        '''
        # Convert from degrees per second to encoder velocity
        vel = int(vel * 41.7)

        # Write to servo
        self.write4ByteData(self.ADDR_X_GOAL_VELOCITY, vel)

    def get_vel(self) -> float:
        # Retrieve current encoder velocity as unsigned 32-bit integer
        vel = self.read4ByteData(self.ADDR_X_PRESENT_VELOCITY)

        # Convert to signed 32-bit integer
        vel = DynaController.to_signed32(vel)

        # Convert to degrees per second
        vel = vel / 41.7

        return vel
    
    def set_torque(self, torque: bool = False) -> None:
        '''
        Enable/disable servo torque.

        Parameters:
        - torque (bool): True to enable torque, False to disable.
        '''
        self.write1ByteData(self.ADDR_X_TORQUE_ENABLE, int(torque))

        check = self.read1ByteData(self.ADDR_X_TORQUE_ENABLE)

        if check == int(torque):
            return True
        else:
            return False

    def set_op_mode(self, mode: int = 3) -> bool:
        '''
        Set servo operating mode.

        Parameters:
        - mode (int): Operating mode to set. 3 => position control, 1 => velocity control.
        '''

        # Set the operating mode
        self.write1ByteData(self.ADDR_X_OPERATING_MODE, mode)

        # Verify that the operating mode was set correctly
        check = self.read1ByteData(self.ADDR_X_OPERATING_MODE)

        # Return True if the operating mode was set correctly, False otherwise
        if check == mode:
            return True
        else:
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

    def calibrate_position(self):

        self.write4ByteData(self.ADDR_X_GOAL_POSITION, 180)

        print("Calibrating position:")
        input("Manually align the servo and press Enter when ready.")

    def close_port(self):
        self.portHandler.closePort()


if __name__ == '__main__':
    dyna = DynaController()

    if dyna.open_port():
        # Port opened
        print("Port opened successfully\n")

        # Set torque to false to allow for EEPROM writes
        print(f"Torque turned off : {dyna.set_torque(False)}")

        print(f"Set operating mode to position: {dyna.set_op_mode(3)}\n")

        print(f"Torque turned on : {dyna.set_torque(True)}")

        print("Moving to 0 degrees\n")
        dyna.set_pos(0)

        time.sleep(3)

        print("Moving to 360 degrees\n")
        dyna.set_pos(360)

        time.sleep(3)

        print("Moving to 180 degrees\n")
        dyna.calibrate_position()

        start_pos = dyna.read4ByteData(dyna.ADDR_X_PRESENT_POSITION)

        # try:
        #     vel = 100
        #     while True:
        #         vel = -vel
        #         dyna.write4ByteData(dyna.ADDR_X_GOAL_VELOCITY, vel)
        #         time.sleep(4)

        #         pres_pos = start_pos - dyna.read4ByteData(dyna.ADDR_X_PRESENT_POSITION)
        #         if pres_pos is not None:
        #             pres_angle = DynaController.to_signed32(pres_pos)
        #             pres_angle = round(360 * (pres_pos / 4096), 3)
        #             print(f"Current position: {pres_angle} degrees")

        #         pres_vel = dyna.read4ByteData(dyna.ADDR_X_PRESENT_VELOCITY)
        #         if pres_vel is not None:
        #             pres_vel = DynaController.to_signed32(pres_vel)
        #             print(f"Current velocity: {pres_vel}\n")

        # except KeyboardInterrupt:
        #     print("Interrupted by user")
        #     dyna.write1ByteData(dyna.ADDR_X_TORQUE_ENABLE, 0)
        #     dyna.close_port()

        # except Exception as e:
        #     print(f"Error during operation: {e}")
        #     dyna.write1ByteData(dyna.ADDR_X_TORQUE_ENABLE, 0)
        #     dyna.close_port()

