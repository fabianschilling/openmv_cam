#!/usr/bin/env python
"""Publishes OpenMV Cam images as ROS messages."""

import sys

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image

from openmv_cam import OpenMVCam


class OpenMVCamNode:

    def __init__(self):

        rospy.init_node('openmv_cam_node', argv=sys.argv)

        self.device = rospy.get_param('~device', default='/dev/ttyACM0')
        self.topic = rospy.get_param('~topic', default='openmv_cam/image_raw')
        self.compressed = rospy.get_param('~compressed', default=False)

        self.image_type = Image
        if self.compressed:
            self.topic += '/compressed'
            self.image_type = CompressedImage

        try:
            self.openmv_cam = OpenMVCam(self.device)
        except Exception:
            rospy.logerr('Serial connection to OpenMV Cam failed: {}'
                         .format(self.device))
            exit()

        self.bridge = CvBridge()

        self.publisher = rospy.Publisher(self.topic, self.image_type,
                                         queue_size=1)
        self.seq = 0

        rospy.loginfo("Relaying '{}' to '{}'".format(self.device, self.topic))

    def read_and_publish_image(self):

        # Read image from camera
        image = self.openmv_cam.read_image()

        # Deduce color/grayscale from image dimensions
        channels = 1 if image.ndim == 2 else 3
        encoding = 'bgr8' if channels == 3 else 'mono8'

        # Convert from BGR to RGB by reversing the channel order
        if channels == 3:
            image = image[..., ::-1]

        # Convert numpy image to (commpressed) ROS image message
        if self.compressed:
            image_msg = self.bridge.cv2_to_compressed_imgmsg(image)
        else:
            image_msg = self.bridge.cv2_to_imgmsg(image, encoding=encoding)

        # Add timestamp and sequence number (empty by default)
        image_msg.header.stamp = rospy.Time.now()
        image_msg.header.seq = self.seq
        self.seq += 1

        self.publisher.publish(image_msg)


def main():

    openmv_cam_node = OpenMVCamNode()

    while not rospy.is_shutdown():
        try:
            openmv_cam_node.read_and_publish_image()
        except Exception:
            pass  # Fail silently


if __name__ == '__main__':
    main()
