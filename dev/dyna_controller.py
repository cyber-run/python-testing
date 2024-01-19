from dynamixel_sdk import *  # Uses Dynamixel SDK library
import logging
import time

class DynaController:
    def __init__(self, com_port: str = 'COM5', baud_rate: int = 115200) -> None:
        # EEPROM addresses for X-series:
        self.X_TORQUE_ENABLE = 64       # Torque enable
        self.X_OP_MODE = 11             # Operating mode
        self.X_VEL_LIM = 44             # Velocity limit
        self.X_SET_VEL = 104            # Set velocity
        self.X_SET_POS = 116            # Set position
        self.X_GET_VEL = 128            # Get velocity
        self.X_GET_POS = 132            # Get position

        # Protocol version : # X-series uses protocol version 2.0
        self.PROTOCOL_VERSION = 2.0

        # Dynamixel motors IDs (crossref with wizard)
        self.pan_id = int(1)
        self.tilt_id = int(2)
        self.zoom_id = int(3)

        # COM and U2D2 params
        self.baud = baud_rate
        self.com = com_port

        self.port_handler = PortHandler(self.com)
        self.packet_handler = PacketHandler(self.PROTOCOL_VERSION)

        # Initialize GroupSyncWrite instance
        self.pos_sync_write = GroupSyncWrite(self.port_handler, self.packet_handler, self.X_SET_POS, 4)
        

    def open_port(self) -> bool:
        '''
        Open serial port for communication with servo.
        
        Returns:
        - bool: True if port opened successfully, False otherwise.'''

        try:
            if self.port_handler.openPort():
                logging.info("Succeeded to open the port")
            else:
                logging.error("Failed to open the port")
                return False

            if self.port_handler.setBaudRate(self.baud):
                logging.info("Succeeded to change the baudrate")
            else:
                logging.error("Failed to change the baudrate")
                return False
            return True
        except Exception as e:
            print(f"Error opening port: {e}")
            return False

    def set_pos(self, motor_id: int = 1, pos: float = 180) -> None:
        '''
        Set servo position in degrees for a specified motor.

        Parameters:
        - motor_id (int): ID of the motor.
        - pos (float): Desired position in degrees.
        '''
        # Convert from degrees to encoder position
        pos = int(pos * 4095 / 360)
        logging.debug(f"Setting motor {motor_id} position to {pos} ticks")
        # Write to servo
        self.write4ByteData(motor_id, self.X_SET_POS, pos)

    def set_sync_pos(self, pan_pos: float = 180, tilt_pos: float = 180) -> None:
        '''
        Set servo position synchronously for both motors.
        
        Parameters:
        - pan_pos (float): Desired pan position in degrees.
        - tilt_pos (float): Desired tilt position in degrees.
        '''
        # Convert from degrees to encoder position
        pan_pos = int(pan_pos * 4095 / 360)
        tilt_pos = int(tilt_pos * 4095 / 360)

        logging.debug(f"Setting pan position to {pan_pos} ticks")
        logging.debug(f"Setting tilt position to {tilt_pos} ticks")

        # Allocate goal positions value into byte array
        pan_pos = [DXL_LOBYTE(DXL_LOWORD(pan_pos)), DXL_HIBYTE(DXL_LOWORD(pan_pos)), DXL_LOBYTE(DXL_HIWORD(pan_pos)), DXL_HIBYTE(DXL_HIWORD(pan_pos))]
        tilt_pos = [DXL_LOBYTE(DXL_LOWORD(tilt_pos)), DXL_HIBYTE(DXL_LOWORD(tilt_pos)), DXL_LOBYTE(DXL_HIWORD(tilt_pos)), DXL_HIBYTE(DXL_HIWORD(tilt_pos))]

        # Add the write data to the syncwrite parameter storage
        dxl_addparam_result = self.pos_sync_write.addParam(self.pan_id, pan_pos)
        if dxl_addparam_result != True:
            logging.error("[ID:%03d] groupSyncWrite addparam failed" % self.pan_id)
            quit()

        # Add the write data to the syncwrite parameter storage
        dxl_addparam_result = self.pos_sync_write.addParam(self.tilt_id, tilt_pos)
        if dxl_addparam_result != True:
            logging.error("[ID:%03d] groupSyncWrite addparam failed" % self.tilt_id)
            quit()

        # Syncwrite goal position
        x = self.pos_sync_write.txPacket()

        # Clear syncwrite parameter storage
        self.pos_sync_write.clearParam()

    def get_pos(self, motor_id: int = 1) -> float:
        '''
        Retrieve the current position of a specified motor in degrees.

        Parameters:
        - motor_id (int): ID of the motor.

        Returns:
        - float: Current position in degrees.
        '''
        # Retrieve current encoder position as unsigned 32-bit integer
        pos = self.read4ByteData(motor_id, self.X_GET_POS)
        if pos is None:
            return None
        # Convert to degrees
        pos = round(360 * (pos / 4095), 3)
        return pos

    def set_vel(self, motor_id: int = 1, vel: float = 0) -> None:
        '''
        Set servo velocity for a specified motor in degrees per second.

        Parameters:
        - motor_id (int): ID of the motor.
        - vel (float): Desired velocity in degrees per second.
        '''
        # Convert from degrees per second to encoder velocity
        vel = int(vel * 41.7)
        # Write to servo
        self.write4ByteData(motor_id, self.X_SET_VEL, vel)

    def get_vel(self, motor_id: int = 1) -> float:
        '''
        Retrieve the current velocity of a specified motor in degrees per second.

        Parameters:
        - motor_id (int): ID of the motor.

        Returns:
        - float: Current velocity in degrees per second.
        '''
        # Retrieve current encoder velocity as unsigned 32-bit integer
        vel = self.read4ByteData(motor_id, self.X_GET_VEL)
        if vel is None:
            return None
        # Convert to degrees per second
        vel = vel / 41.7
        return vel

    def set_torque(self, motor_id: int = 1, torque: bool = False) -> None:
        '''
        Enable/disable servo torque.

        Parameters:
        - torque (bool): True to enable torque, False to disable.
        '''
        self.write1ByteData(motor_id, self.X_TORQUE_ENABLE, int(torque))

        check = self.read1ByteData(motor_id, self.X_TORQUE_ENABLE)

        if check == int(torque):
            return True
        else:
            return False

    def get_torque(self, motor_id: int = 1) -> bool:
        '''
        Get current torque status.

        Returns:
        - bool: True if torque is enabled, False otherwise.
        '''
        torque = self.read1ByteData(motor_id, self.X_TORQUE_ENABLE)

        if torque == 1:
            return True
        else:
            return False

    def set_op_mode(self, motor_id: int = 1, mode: int = 3) -> bool:
        '''
        Set servo operating mode.

        Parameters:
        - mode (int): Operating mode to set. 3 => position control, 1 => velocity control.
        '''
        # Disable torque to set operating mode
        self.set_torque(motor_id, False)

        # Set the operating mode
        self.write1ByteData(motor_id, self.X_OP_MODE, mode)

        # Verify that the operating mode was set correctly
        check = self.read1ByteData(motor_id, self.X_OP_MODE)

        # Re-enable torque
        self.set_torque(motor_id, True)

        # Return True if the operating mode was set correctly, False otherwise
        if check == mode:
            return True
        else:
            return False

    def get_op_mode(self, motor_id: int = 1) -> int:
        '''
        Get current operating mode.

        Returns:
        - int: Current operating mode. 3 => position control, 1 => velocity control.
        '''
        op_mode = self.read1ByteData(motor_id, self.X_OP_MODE)

        return op_mode

    def write1ByteData(self, motor_id, address, value):
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, address, value)
        if dxl_comm_result != COMM_SUCCESS:
            logging.debug(self.packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            logging.error(self.packet_handler.getRxPacketError(dxl_error))

    def write4ByteData(self, motor_id, address, value):
        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(self.port_handler, motor_id, address, value)
        if dxl_comm_result != COMM_SUCCESS:
            logging.debug(self.packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            logging.error(self.packet_handler.getRxPacketError(dxl_error))

    def read1ByteData(self, motor_id, address):
        dxl_data, dxl_comm_result, dxl_error = self.packet_handler.read1ByteTxRx(self.port_handler, motor_id, address)
        if dxl_comm_result != COMM_SUCCESS:
            logging.debug(self.packet_handler.getTxRxResult(dxl_comm_result))
            return None
        elif dxl_error != 0:
            logging.error(self.packet_handler.getRxPacketError(dxl_error))
            return None
        else:
            return dxl_data

    def read4ByteData(self, motor_id, address):
        dxl_data, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(self.port_handler, motor_id, address)
        if dxl_comm_result != COMM_SUCCESS:
            logging.debug(self.packet_handler.getTxRxResult(dxl_comm_result))
            return None
        elif dxl_error != 0:
            logging.error(self.packet_handler.getRxPacketError(dxl_error))
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
        self.port_handler.closePort()


if __name__ == '__main__':
    # Initialise Dynamixel controller object and open port
    dyna = DynaController()
    dyna.open_port()

    # Set operating mode of both to position control
    dyna.set_op_mode(dyna.pan_id, 3)
    dyna.set_op_mode(dyna.tilt_id, 3)
    
    pos = 0
    direction = 1  # Determines whether the position is increasing or decreasing

    # Example usage with motor IDs
    try:
        while True:
            # Set position for each motor
            dyna.set_sync_pos(pos, pos)

            # Update position
            pos += direction
            if pos >= 360 or pos <= 0:
                direction *= -1  # Change direction at 0 and 360 degrees

            time.sleep(0.01)  # Sleep for a short duration

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        dyna.close_port()
        print("Port closed.")
        exit()
        
