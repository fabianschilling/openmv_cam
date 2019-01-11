#!/usr/bin/env python
"""Reads images from OpenMV Cam serial port and publishes them as ROS messages.

Source:
    https://github.com/openmv/openmv/blob/master/scripts/examples/02-Board-Control/usb_vcp.py
"""
from __future__ import absolute_import, division, print_function

import io
import os
import struct
import sys

import numpy as np
import rospy
import serial
from cv_bridge import CvBridge
from PIL import Image as PILImage
from sensor_msgs.msg import Image, CompressedImage


class OpenMVCam:

    def __init__(self):

        self.node_name = 'openmv_cam'
        rospy.init_node(self.node_name, argv=sys.argv)

        self.device = rospy.get_param('~device', default='/dev/ttyACM0')

        try:
            self.port = serial.Serial(self.device, baudrate=115200,
                                      bytesize=serial.EIGHTBITS,
                                      parity=serial.PARITY_NONE,
                                      xonxoff=False, rtscts=False,
                                      stopbits=serial.STOPBITS_ONE,
                                      timeout=None, dsrdtr=True)
        except serial.serialutil.SerialException:
            rospy.logerr('Could not open port {}'.format(self.device))
            exit()

        self.bridge = CvBridge()
        self.image_type = Image

        self.topic = rospy.get_param('~topic', default='openmv_cam/image_raw')
        self.compressed = rospy.get_param('~compressed', default=False)

        # If compression
        if self.compressed:
            self.topic = os.path.join(self.topic, 'compressed')
            self.image_type = CompressedImage

        self.publisher = rospy.Publisher(self.topic, self.image_type,
                                         queue_size=1)

        # Important: reset buffers for reliabile restarts of this node!
        self.port.reset_input_buffer()
        self.port.reset_output_buffer()

        rospy.loginfo("Relaying '{}' to '{}'".format(self.device, self.topic))

    def read_and_publish_image(self):

        # Sending 'snap' command causes camera to take snapshot
        self.port.write('snap')
        self.port.flush()

        # Read 'size' bytes from serial port
        size = struct.unpack('<L', self.port.read(4))[0]
        image_data = self.port.read(size)
        image = np.array(PILImage.open(io.BytesIO(image_data)))

        # Deduce BGR/gray from image dimensions
        channels = 1 if image.ndim == 2 else 3
        encoding = 'bgr8' if channels == 3 else 'mono8'

        # Convert from BGR to RGB by reversing the channel order
        if channels == 3:
            image = image[..., ::-1]

        # Convert numpy image to (commpressed) ROS image message
        if self.compressed:
            ros_image = self.bridge.cv2_to_compressed_imgmsg(image)
        else:
            ros_image = self.bridge.cv2_to_imgmsg(image, encoding=encoding)

        self.publisher.publish(ros_image)


def main():

    openmv_cam = OpenMVCam()

    while not rospy.is_shutdown():
        try:
            openmv_cam.read_and_publish_image()
        except serial.serialutil.SerialException:
            pass  # Fail silently


if __name__ == '__main__':
    main()
