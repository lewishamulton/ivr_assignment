#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera2 to a topic named image_topic2
    self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)


    #intialises a publisher for the joint 3 angle calculated from camera 3
    self.est2_joint3_pub = rospy.Publisher("/robot/joint3_c2_estimated",Float64,queue_size=10)
    # initialises a subscriber for the joint 2 angle calculated from camera 1
    self.image_joint2_est = rospy.Subscriber("/robot/joint2_estimated",Float64,self.get_camera1_joint2)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()

    #start time for robot
    self.time_trajectory = rospy.get_time()

    #estimated joint agles from camera1
    self.est1_joint2 = 0.0





  def defineSinusoidalTrajectory(self):
    #get current time
    cur_time = np.array([rospy.get_time() - self.time_trajectory])
    print("Time:",cur_time)
    joint2_trajectory = float(np.pi/2*np.sin(np.pi/15*cur_time))
    joint3_trajectory = float(np.pi/2*np.sin(np.pi/18*cur_time))
    joint4_trajectory = float(np.pi/2*np.sin(np.pi/20*cur_time))
    return np.array([joint2_trajectory,joint3_trajectory,joint4_trajectory])

  def detect_blue(self,image):
    mask = cv2.inRange(image,(100,0,0),(255,0,0))
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.dilate(mask,kernel,iterations=3)
    cv2.imwrite('blue_copy.png', mask)
    M = cv2.moments(mask)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    return np.array([cx,cy])

  def detect_green(self,image):
    mask = cv2.inRange(image,(0,100,0),(0,255,0))
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.dilate(mask,kernel,iterations=3)
    M = cv2.moments(mask)
    try:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        return np.array([cx,cy])
    #blob is blocked or out of camera view = zerodivision
    except ZeroDivisionError:
        #switch camera view fro camera 1
        print("Switch camera view")

  #convert pixel to meters for link 3
  def pixelToMeter(self,image):
    blue_circle = self.detect_blue(image)
    green_circle = self.detect_green(image)
    dist = np.sum((green_circle - blue_circle)**2)
    return 3.5/np.sqrt(dist)

  #gets estimate of joint 3 angle
  def detect_joint3(self,image,j2_angle):
    p = self.pixelToMeter(image)
    green_blob = self.detect_green(image)
    blue_blob = self.detect_blue(image)

    joint3_pos = p*(blue_blob - green_blob)

    j3_angle = np.arctan2(joint3_pos[0],joint3_pos[1])
    return j3_angle

  def get_camera1_joint2(self,data):
    self.est1_joint2 = data.data

  # Recieve data, process it, and publish
  def callback2(self,data):
    # Recieve the image
    try:
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)


    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)
    im2=cv2.imshow('window2', self.cv_image2)
    cv2.waitKey(1)


    est_j3 = self.detect_joint3(self.cv_image2,self.est1_joint2)
    self.est2_joint3_pub.publish(est_j3)

    # Publish the results
    try:
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
    except CvBridgeError as e:
      print(e)

# call the class
def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
