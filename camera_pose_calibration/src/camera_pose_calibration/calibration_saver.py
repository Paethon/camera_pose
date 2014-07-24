#!/usr/bin/python

"""
Subscribes to the topic which provides the calibration information from
camera_pose_calibration and writes it to an XML file
"""
from __future__ import print_function

import roslib
roslib.load_manifest('camera_pose_calibration')

import rospy
import argparse
import datetime
import xml.etree.ElementTree as ET
from camera_pose_calibration.msg import CameraCalibration

parser = argparse.ArgumentParser()
parser.add_argument('xmlFile',
                    help='Destination file for calibration data')
parser.add_argument('ROS', help='Catches the ROS arguments if provided by roslaunch',
                    nargs="+")
args = parser.parse_args()
filename = args.xmlFile
# filename = 'test.xml'

def callback(msg):
    # Build XML tree
    root = ET.Element('camera_calibration_data')
    root.set('message_count', str(msg.m_count))
    root.set('date', datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'))
    for i in range(len(msg.camera_id)):
        camera = ET.SubElement(root, 'camera')
        camera.set('camera_id', msg.camera_id[i])
        position = ET.SubElement(camera, 'position')
        ET.SubElement(position, 'x').text = str(msg.camera_pose[i].position.x)
        ET.SubElement(position, 'y').text = str(msg.camera_pose[i].position.y)
        ET.SubElement(position, 'z').text = str(msg.camera_pose[i].position.z)
        orientation = ET.SubElement(camera, 'orientation')
        ET.SubElement(orientation, 'x').text = str(msg.camera_pose[i].orientation.x)
        ET.SubElement(orientation, 'y').text = str(msg.camera_pose[i].orientation.y)
        ET.SubElement(orientation, 'z').text = str(msg.camera_pose[i].orientation.z)
        ET.SubElement(orientation, 'w').text = str(msg.camera_pose[i].orientation.w)

    # Save XML file
    tree = ET.ElementTree(root)
    tree.write(filename, xml_declaration=True)
    print("Saved calibration data")


def listener():
    rospy.init_node('Calibration_writer')
    rospy.Subscriber("/camera_calibration", CameraCalibration, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
