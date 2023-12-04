from dynamixel_sdk import *  # Uses Dynamixel SDK library
import logging
import time

class DynaController:
    def __init__(self, com_port: str = 'COM5', baud_rate: int = 115200, servo_id: int = 1) -> None:
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
        self.DXL_ID = servo_id  # Dynamixel ID
        self.BAUDRATE = baud_rate  # Dynamixel default baudrate
        self.DEVICENAME = com_port  # Port name

        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        while self.open_port() is False:
            logging.info("Trying to open port...")
            time.sleep(5)
        
        logging.info("Port opened successfully")

    def open_port(self) -> bool:
        '''
        Open serial port for communication with servo.
        
        Returns:
        - bool: True if port opened successfully, False otherwise.'''

        try:
            if self.portHandler.openPort():
                logging.info("Succeeded to open the port")
            else:
                logging.error("Failed to open the port")
                return False

            if self.portHandler.setBaudRate(self.BAUDRATE):
                logging.info("Succeeded to change the baudrate")
            else:
                logging.error("Failed to change the baudrate")
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

        logging.info(f"Setting position to {pos} ticks")

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

    def get_torque(self) -> bool:
        '''
        Get current torque status.

        Returns:
        - bool: True if torque is enabled, False otherwise.
        '''
        torque = self.read1ByteData(self.ADDR_X_TORQUE_ENABLE)

        if torque == 1:
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

    def get_op_mode(self) -> int:
        '''
        Get current operating mode.

        Returns:
        - int: Current operating mode. 3 => position control, 1 => velocity control.
        '''
        op_mode = self.read1ByteData(self.ADDR_X_OPERATING_MODE)

        return op_mode

    def write1ByteData(self, address, value):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, address, value)
        if dxl_comm_result != COMM_SUCCESS:
            logging.info(self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            logging.error(self.packetHandler.getRxPacketError(dxl_error))

    def write4ByteData(self, address, value):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID, address, value)
        if dxl_comm_result != COMM_SUCCESS:
            logging.info(self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            logging.error(self.packetHandler.getRxPacketError(dxl_error))

    def read1ByteData(self, address):
        dxl_data, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL_ID, address)
        if dxl_comm_result != COMM_SUCCESS:
            logging.info(self.packetHandler.getTxRxResult(dxl_comm_result))
            return None
        elif dxl_error != 0:
            logging.error(self.packetHandler.getRxPacketError(dxl_error))
            return None
        else:
            return dxl_data

    def read4ByteData(self, address):
        dxl_data, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, address)
        if dxl_comm_result != COMM_SUCCESS:
            logging.info(self.packetHandler.getTxRxResult(dxl_comm_result))
            return None
        elif dxl_error != 0:
            logging.error(self.packetHandler.getRxPacketError(dxl_error))
            return None
        else:
            return dxl_data

    @staticmethod
    def to_signed32(n):
        n = n & 0xffffffff
        return (n ^ 0x80000000) - 0x80000000

    def calibrate_position(self):

        self.set_pos(180)

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

        print("\nMoving to 0 degrees:")
        dyna.set_pos(0)
        time.sleep(2)
        print(f"Current position: {dyna.get_pos()}")

        print("\nMoving to 360 degrees:")
        dyna.set_pos(360)
        time.sleep(2)
        print(f"Current position: {dyna.get_pos()}")

        print("\nMoving to 180 degrees:")
        dyna.calibrate_position()
        time.sleep(2)
        print(f"Current position: {dyna.get_pos()}")

        start_pos = dyna.read4ByteData(dyna.ADDR_X_PRESENT_POSITION)
