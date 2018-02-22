#!/usr/bin/python
# -*- coding: utf-8 -*-
import sys
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sensor_msgs.msg import Image

class dense_optical_flow():
    def __init__(self, imgtopic):

        self.old_frame = None
        self.new_frame = None
        self.hsv = None

        self.image_sub = rospy.Subscriber(imgtopic,Image,
                                      self.img_callback)


    def img_callback(self,data):
        """ Receives the Image topic message and converts it from sensor_msgs
        to cv image using cv_bridge.
        """
        bridge = CvBridge()
        try:
            # Read and convert data
            color_frame = bridge.imgmsg_to_cv2(data, "bgr8")
            small = cv2.resize(color_frame,(0,0), fx=0.5, fy=0.5)
            self.new_frame = cv2.cvtColor( small,
                                          cv2.COLOR_BGR2GRAY)
            if self.old_frame == None and self.hsv == None:
                self.old_frame = self.new_frame
                self.hsv = np.zeros_like(small)
                self.hsv[...,1] = 255

            self.calc_optical_flow()
        except CvBridgeError as e:
            print(e)

    def calc_optical_flow(self):
        # Calculate the dense optical flow
        flow = cv2.calcOpticalFlowFarneback(self.old_frame, self.new_frame, 
                                            None, 0.5, 3, 15, 3, 5, 1.2, 0)
        # Obtain the flow magnitude and direction angle
        mag, ang = cv2.cartToPolar(flow[...,0], flow[...,1])

        print mag

        print ang

        print '-----------------------------------'

        # Update the color image
        self.hsv[...,0] = ang*180/np.pi/2
        self.hsv[...,2] = cv2.normalize(mag,None,0,255,cv2.NORM_MINMAX)
        bgr = cv2.cvtColor(self.hsv,cv2.COLOR_HSV2BGR)
        
        cv2.imshow('frame2',bgr)
        cv2.waitKey(3)
        
        self.old_frame = self.new_frame
    
    def lucas_kanade(self):
        pass


def main(args):
  rospy.init_node('dense_optical_flow', anonymous=True)
  of = dense_optical_flow('/BlueRov2/camera/image_raw')
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)