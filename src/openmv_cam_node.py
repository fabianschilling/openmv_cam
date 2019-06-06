#!/usr/bin/env python
"""Publishes OpenMV Cam images as ROS messages."""

import os
import sys

import rospkg
import rospy
from camera_info_manager import CameraInfoManager
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

from openmv_cam import OpenMVCam

CAMERA_NAME = 'openmv_cam'
DEFAULT_IMAGE_TOPIC = '{}/image_raw'.format(CAMERA_NAME)
DEFAULT_CAMERA_TOPIC = '{}/camera_info'.format(CAMERA_NAME)


class OpenMVCamNode:

    def __init__(self):

        rospy.init_node('{}_node'.format(CAMERA_NAME), argv=sys.argv)

        self.device = rospy.get_param('~device', '/dev/ttyACM0')
        self.image_topic = rospy.get_param('~image', DEFAULT_IMAGE_TOPIC)
        self.camera_topic = rospy.get_param('~camera', DEFAULT_CAMERA_TOPIC)
        self.compressed = rospy.get_param('~compressed', False)
        self.calibration = rospy.get_param('~calibration', None)

        if not os.path.exists(self.calibration):
            rospy.logerr('Calibration not found: {}'.format(self.calibration))
            exit()

        url = 'file://' + self.calibration
        self.manager = CameraInfoManager(cname=CAMERA_NAME, url=url,
                                         namespace=CAMERA_NAME)

        self.manager.loadCameraInfo()
        self.camera_info = self.manager.camera_info

        self.image_type = Image
        if self.compressed:
            self.topic += '/compressed'
            self.image_type = CompressedImage

        self.openmv_cam = OpenMVCam(self.device)

        self.bridge = CvBridge()

        self.image_publisher = rospy.Publisher(self.image_topic,
                                               data_class=self.image_type,
                                               queue_size=1)
        self.camera_publisher = rospy.Publisher(self.camera_topic,
                                                data_class=CameraInfo,
                                                queue_size=1)
        self.seq = 0

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

        self.image_publisher.publish(image_msg)
        camera_msg = self.camera_info
        camera_msg.header = image_msg.header
        self.camera_publisher.publish(camera_msg)

        if self.seq == 0:
            rospy.loginfo("Publishing images from OpenMV Cam at '{}' "
                          .format(self.device))
        self.seq += 1


def main():

    openmv_cam_node = OpenMVCamNode()

    while not rospy.is_shutdown():
        openmv_cam_node.read_and_publish_image()


if __name__ == '__main__':
    main()
