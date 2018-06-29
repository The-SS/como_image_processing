#!/usr/bin/env python
'''
We get a raw image from the connected webcam and publish its pixels to
a topic called /cam/raw

Author:
    Sleiman Safaoui

March 23, 2018
'''
from __future__ import print_function

#sys files import
import cv2
import os

#ros imports
import rospy
import roslib
#from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ImagePublisher:

    def __init__(self):
        self.bridge = CvBridge()
        self.keep_running = True
        self.cam_pub = rospy.Publisher("/cam/raw", Image, queue_size=1)
        self.img_mode = ''
        self.cam_image_raw = []

    def get_img(self, cam, img_mode):
        if ((img_mode != 'color') & (img_mode != 'gray')) :
            #print('Image mode not valid')
            rospy.logfatal('Image mode not valid')
            return
        ret, self.cam_image_raw = cam.read() #fetch raw camera image
        if ret:
            try:
                if img_mode == 'color':
                    self.cam_pub.publish(self.bridge.cv2_to_imgmsg(self.cam_image_raw, "bgr8")) # publish colored image
                else: # gray image
                    img_gray = cv2.cvtColor(self.cam_image_raw, cv2.COLOR_BGR2GRAY)
                    self.cam_pub.publish(self.bridge.cv2_to_imgmsg(img_gray, "mono8")) # publish grayscale image
            except CvBridgeError as e:
                print(e)
                rospy.logerr(e)



def main():
    rospy.init_node("cam_bridge", anonymous=True) #initialize ros node
    rate = rospy.Rate(30) #set publishing rate
    
    nodename = '/cam_bridge'
    
    image_mode = rospy.get_param(nodename + '/imgMode')
    cam_path = rospy.get_param(nodename + '/camPath')
    res_w = rospy.get_param(nodename + '/resolution' + '/width')
    res_h = rospy.get_param(nodename + '/resolution' + '/height')
    
    os.system('v4l2-ctl -d ' + cam_path + ' -v width=' + str(res_w) + ',height=' + str(res_h))
    
    cam = cv2.VideoCapture() #create cam object
    cam.open(cam_path) #start cam based on cam_path
    #cam.open('/dev/video6')

    ip = ImagePublisher() #create ImagePublisher object

    try:
        while not rospy.is_shutdown():
            ip.get_img(cam, image_mode) #get image
            rate.sleep() #sleep for rest of time
    except KeyboardInterrupt:
        #print("shutting down ros")
        rospy.logfatal("Keyboard Interrupt. Shutting down cam_bridge node")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logfatal("ROS Interrupt. Shutting down cam_bridge node")
        pass
