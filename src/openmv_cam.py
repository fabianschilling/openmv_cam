"""Reads images from OpenMV Cam serial port.

Source:
    https://github.com/openmv/openmv/blob/master/scripts/examples/02-Board-Control/usb_vcp.py
"""

import io
import struct

import numpy as np
import serial
from PIL import Image as PILImage


class OpenMVCam:

    def __init__(self, device='/dev/ttyACM0'):
        """Reads images from OpenMV Cam

        Args:
            device (str): Serial device

        Raises:
            serial.SerialException

        """
        self.port = serial.Serial(device, baudrate=115200,
                                  bytesize=serial.EIGHTBITS,
                                  parity=serial.PARITY_NONE,
                                  xonxoff=False, rtscts=False,
                                  stopbits=serial.STOPBITS_ONE,
                                  timeout=None, dsrdtr=True)

        # Important: reset buffers for reliabile restarts of OpenMV Cam
        self.port.reset_input_buffer()
        self.port.reset_output_buffer()

    def read_image(self):
        """Read image from OpenMV Cam

        Returns:
            image (ndarray): Image

        Raises:
            serial.SerialException
        """

        # Sending 'snap' command causes camera to take snapshot
        self.port.write('snap')
        self.port.flush()

        # Read 'size' bytes from serial port
        size = struct.unpack('<L', self.port.read(4))[0]
        image_data = self.port.read(size)
        image = np.array(PILImage.open(io.BytesIO(image_data)))

        return image
