from threading import Thread
from scipy.io import savemat
import logging
import asyncio
import math
import qtm


class MoCap(Thread):

    def __init__(self, position=[0,0], rotation=0, height=0, qtm_ip="192.168.100.1", stream_type='6d'):
        """
        Constructs QtmWrapper object
        :param position: 6D body position
        :param rotation: 6D body rotation
        :param height: For drone altitude control
        :param qtm_ip: IP of QTM instance, but doesn't seem to matter
        :param stream_type: Specify components to receive,
                            see: https://github.com/qualisys/qualisys_python_sdk/blob/master/qtm/qrt.py
        """

        Thread.__init__(self)

        # QTM Connection vars
        self.qtm_ip = qtm_ip
        self.stream_type = stream_type
        self._connection = None
        self._stay_open = True

        # Kinematic data vars
        self.state = [0, 0, 0, 0, 0, 0]
        self.position = position
        self.rotation = rotation
        self.lost = False

        self.start()

    def run(self) -> None:
        """
        Run QTM wrapper coroutine.
        """
        asyncio.run(self._life_cycle())

    async def _life_cycle(self) -> None:
        """
        QTM wrapper coroutine.
        """
        await self._connect()
        while(self._stay_open):
            await asyncio.sleep(1)
        await self._close()

    async def _connect(self) -> None:
        """
        Connect to QTM machine.
        """
        # Establish connection
        logging.info('[QTM] Connecting to QTM at %s', self.qtm_ip)
        self._connection = await qtm.connect(self.qtm_ip)

        if self.stream_type == '6d':
            # Register index of body for 6D tracking
            params_xml = await self._connection.get_parameters(parameters=['6d'])

            # Assign 6D streaming callback
            await self._connection.stream_frames(components=['6DEuler'], on_packet=self._on_packet)

        if self.stream_type == '3d':
            # Register index of body for 6D tracking
            params_xml = await self._connection.get_parameters(parameters=['3d'])

            # Assign 6D streaming callback
            await self._connection.stream_frames(components=["3dnolabels"], on_packet=self._on_packet)

    def _on_packet(self, packet) -> None:
        """
        Process 6D packet stream into Pose object and pass on.
        Parameters
        ----------
        packet : QRTPacket
            Incoming packet from QTM
        """
        if self.stream_type == '6d':
            # Extract new 6D component from packet
            header, new_component = packet.get_6d_euler()

            # If no new component: mark as lost and return from function
            if not new_component:
                logging.warning('[QTM] 6DoF rigid body not found.')
                self.lost = True
                return

            pos, rot = new_component[0]
            self.position = [pos.x, pos.y, pos.z]
            self.rotation = rot.a3
            self.state = [pos.x, pos.y, pos.z, rot.a1, rot.a2, rot.a3]
            self.lost = False

        elif self.stream_type == '3d': 
            # Extract new unlabelled 3d component from packet
            header, new_component = packet.get_3d_markers_no_label()

            # If no new component: mark as lost and return from function
            if not new_component:
                logging.warning('[QTM] 3D Unlabelled marker not found.')
                self.lost = True
                return

            pos = new_component[0]
            self.position = [pos.x, pos.y, pos.z]
            self.lost = False

    async def _close(self) -> None:
        """
        End lifecycle by disconnecting from QTM machine.
        """
        await self._connection.stream_frames_stop()
        self._connection.disconnect()

    def close(self) -> None:
        """
        Stop QTM wrapper thread.
        """
        self._stay_open = False
        self.join()


class DataLogger:
    def __init__(self):
        self.height = []
        self.velocity = []
        self.timeStamp = []
        self.ref = []

    def log(self, data):
        self.velocity.append(data[0])
        self.height.append(data[1])
        self.timeStamp.append(data[2])
        self.ref.append(data[3])

    def save_file(self, file_path='data.mat'):
        mdic = {'height': self.height, 'velocity': self.velocity, 'time_stamp': self.timeStamp, 'estimate_height': self.ref}
        savemat(file_path, mdic)


