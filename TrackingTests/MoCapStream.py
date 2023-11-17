from threading import Thread
import asyncio
import qtm
import scipy.io


class MoCap(Thread):

    def __init__(self, position=0, rotation=0, height=0, marker=0, qtm_ip="127.0.0.1", stream_type='6d'):
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

        self.qtm_ip = qtm_ip
        self.height = height
        self.position = position
        self.rotation = rotation
        self.marker = marker
        self.frame = 0
        self.stream_type = stream_type

        self.tracking_loss = 0

        self._body_idx = None
        self._connection = None
        self._stay_open = True

        self.start()

    def run(self) -> None:
        """
        Run QTM wrapper coroutine.
        """
        asyncio.run(self._life_cycle())

    async def _life_cycle(self):
        """
        QTM wrapper coroutine.
        """
        await self._connect()
        while(self._stay_open):
            await asyncio.sleep(1)
        await self._close()

    async def _connect(self):
        """
        Connect to QTM machine.
        """
        # Establish connection
        print('[QTM] Connecting to QTM at ' + self.qtm_ip)
        self._connection = await qtm.connect(self.qtm_ip)

        if self.stream_type == '6d':
            # Register index of body for 6D tracking
            params_xml = await self._connection.get_parameters(parameters=['6d'])

            # Assign 6D streaming callback
            await self._connection.stream_frames(components=["6D"], on_packet=self._on_packet)

        if self.stream_type == '3d_unlabelled':
            # Register index of body for 6D tracking
            params_xml = await self._connection.get_parameters(parameters=['3d'])

            # Assign 6D streaming callback
            await self._connection.stream_frames(components=["3dnolabels"], on_packet=self._on_packet)

        if self.stream_type == '3d_labelled':
            # Register index of body for 6D tracking
            params_xml = await self._connection.get_parameters(parameters=['3d'])

            # Assign 6D streaming callback
            await self._connection.stream_frames(components=["3d"], on_packet=self._on_packet)

    def _on_packet(self, packet):
        """
        Process 6D packet stream into Pose object and pass on.
        Parameters
        ----------
        packet : QRTPacket
            Incoming packet from QTM
        """
        if self.stream_type == '6d':
            # Extract 6D component from packet
            header, component_6d = packet.get_6d()
            self.frame = packet.framenumber

            # Increment tracking loss if no component found
            if component_6d is None:
                print('[QTM] Packet without 6D component! Moving on...')
                self.tracking_loss += 1
                return

            pos, rot = component_6d[0]
            self.position = pos
            self.height = pos.z
            self.rotation = rot

        elif self.stream_type == "3d_unlabelled":
            # Extract unlabelled 3d component from packet
            header, component_3d = packet.get_3d_markers_no_label()
            self.frame = packet.framenumber

            # Increment tracking loss if no component found
            if component_3d is None:
                print('[QTM] Packet without 6D component! Moving on...')
                self.tracking_loss += 1
                return
            self.marker = component_3d

        elif self.stream_type == '3d_labelled':
            # Extract unlabelled 3d component from packet
            header, component_3d = packet.get_3d_markers()
            self.frame = packet.framenumber

            # Increment tracking loss if no component found
            if component_3d is None:
                print('[QTM] Packet without 6D component! Moving on...')
                self.tracking_loss += 1
                return
            self.marker = component_3d
        # # Extract relevant body data from 6D component
        # body_6d = component_6d[self._body_idx]
        # # Create Pose object from 6D data
        # pose = Pose.from_qtm_6d(body_6d)
        # # Check validity and pass on
        # if pose.is_valid():
        #     self.on_pose(pose)
        #     self.tracking_loss = 0
        # else:
        #     self.tracking_loss += 1

    async def _close(self):
        """
        End lifecycle by disconnecting from QTM machine.
        """
        await self._connection.stream_frames_stop()
        self._connection.disconnect()

    def close(self):
        """
        Stop QTM wrapper thread.
        """
        self._stay_open = False
        self.join()


class DataLogger:
    def __init__(self):
        self.height = []
        self.actuation = []
        self.timeStamp = []
        self.ref = []

    def log(self, data):
        self.actuation.append(data[0])
        self.height.append(data[1])
        self.timeStamp.append(data[2])
        self.ref.append(data[3])

    def save_file(self, file_path='data.mat'):
        mdic = {'height': self.height, 'thrust': self.actuation, 'time_stamp': self.timeStamp}
        scipy.io.savemat(file_path, mdic)

