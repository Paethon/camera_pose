#!/usr/bin/env python

import roslib; roslib.load_manifest('camera_pose_calibration')
import cv
from cv_bridge import CvBridge, CvBridgeError
import rospy
import threading
from calibration_msgs.msg import Interval
from sensor_msgs.msg import Image

class ImageRenderer:
    def __init__(self, ns):
        self.lock = threading.Lock()
        self.image = None
        self.interval = 0
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(ns+'/image_throttle', Image, self.image_cb)
        self.interval_sub = rospy.Subscriber(ns+'/settled_interval', Interval, self.interval_cb)
        self.max_interval = 1.0


    def image_cb(self, msg):
        with self.lock:
            self.image = msg

    def interval_cb(self, msg):
        with self.lock:
            self.interval = (msg.end - msg.start).to_sec()

    def render(self, window):
        with self.lock:
            if self.image:
                cv.Resize(self.bridge.imgmsg_to_cv(self.image, 'rgb8'), window)
                interval = min(1,(self.interval / self.max_interval))
                cv.Rectangle(window, 
                             (int(0.05*window.width), int(window.height*0.9)), 
                             (int(interval*window.width*0.9+0.05*window.width), int(window.height*0.95)), 
                             (0, interval*255, (1-interval)*255), thickness=-1)
                cv.Rectangle(window, 
                             (int(0.05*window.width), int(window.height*0.9)), 
                             (int(window.width*0.9+0.05*window.width), int(window.height*0.95)), 
                             (0, interval*255, (1-interval)*255))


class Aggregator:
    def __init__(self, ns_list):
        print "Creating aggregator for ", ns_list

        # image
        w = 640
        h = 480
        self.image_out = cv.CreateMat(h, w, cv.CV_8UC3)
        self.pub = rospy.Publisher('aggregated_image', Image)
        self.bridge = CvBridge()

        # create render windows
        layouts = [ (1,1), (2,2), (2,2), (2,2), (3,3), (3,3), (3,3), (3,3), (3,3) ]
        layout = layouts[len(ns_list)-1]
        sub_w = w / layout[0]
        sub_h = h / layout[1]
        self.windows = []
        for j in range(layout[1]):
            for i in range(layout[0]):
                self.windows.append( cv.GetSubRect(self.image_out, (i*sub_w, j*sub_h, sub_w, sub_h) ) )
            
        # create renderers
        self.renderer_list = []
        for ns in ns_list:
            self.renderer_list.append(ImageRenderer(ns))

    def loop(self):
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            for window, render in zip(self.windows, self.renderer_list):
                render.render(window)

            self.pub.publish(self.bridge.cv_to_imgmsg(self.image_out, encoding="passthrough"))
            r.sleep()


def main():
    rospy.init_node('capture_monitor')
    args = rospy.myargv()

    a = Aggregator(args[1:])
    a.loop()



if __name__ == '__main__':
    main()
