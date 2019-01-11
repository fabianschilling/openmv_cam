"""Sends images acquired from the OpenMV Cam via the serial port.

Important:
    This script should be copied to the OpenMV Cam as `main.py`.

Source:
    https://github.com/openmv/openmv/blob/master/scripts/examples/02-Board-Control/usb_vcp.py

"""
import sensor
import ustruct
import pyb

usb_vcp = pyb.USB_VCP()
# Disable USB interrupt (CTRL-C by default) when sending raw data (i.e. images)
# See: https://docs.openmv.io/library/pyb.USB_VCP.html#pyb.USB_VCP.setinterrupt
usb_vcp.setinterrupt(-1)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)  # wait for settings to take effect!

while True:

    command = usb_vcp.recv(4, timeout=5000)

    if command == b'snap':
        image = sensor.snapshot().compress()
        usb_vcp.send(ustruct.pack('<L', image.size()))
        usb_vcp.send(image)
