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
    # initialize a publisher to send images from camera1 to a topic named image_topic1
    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)

    #set up publisher to send joint angles to the robot
    self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command",Float64,queue_size=10)
    self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command",Float64,queue_size=10)
    self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command",Float64,queue_size=10)
    self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command",Float64,queue_size=10)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()

    #start time for robot
    self.time_trajectory = rospy.get_time()


  def defineSinusoidalTrajectory(self):
    #get current time
    cur_time = np.array([rospy.get_time() - self.time_trajectory])
    joint2_trajectory = float(np.pi/2*np.sin(np.pi/15*cur_time))
    joint3_trajectory = float(np.pi/2*np.sin(np.pi/18*cur_time))
    joint4_trajectory = float(np.pi/2*np.sin(np.pi/20*cur_time))
    return np.array([joint2_trajectory,joint3_trajectory,joint4_trajectory])

  def detect_red(self,image):
    mask = cv2.inRange(image,(0,0,100),(0,0,255))
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.dilate(mask,kernel,iterations=3)

    M = cv2.moments(mask)
    try:
        cx = int(M['m10']/M['m00'])
        cy = int(M['01']/M['m00'])
    #blob is blocked or out of camera view = zerodivision
    except ZeroDivisionError:
        #implement chamfer matching here
        print "Needs chamfer matching"

  def detect_green(self,image):
    mask = cv2.inRange(image,(0,100,0),(0,255,0))
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.dilate(mask,kernel,iterations=3)

    M = cv2.moments(mask)
    try:
        cx = int(M['m10']/M['m00'])
        cy = int(M['01']/M['m00'])
    #blob is blocked or out of camera view = zerodivision
    except ZeroDivisionError:
        #implement chamfer matching here
        print "Needs chamfer matching"

  def detect_blue(self,image):
    mask = cv2.inRange(image,(100,0,0),(0,255,0))
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.dilate(mask,kernel,iterations=3)

    M = cv2.moments(mask)
    try:
        cx = int(M['m10']/M['m00'])
        cy = int(M['01']/M['m00'])
    #blob is blocked or out of camera view = zerodivision
    except ZeroDivisionError:
        #implement chamfer matching here
        print "Needs chamfer matching"

  def detect_yellow(self,image):
    mask = cv2.inRange(image,(0,100,100),(0,255,255))
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.dilate(mask,kernel,iterations=3)

    M = cv2.moments(mask)
    try:
        cx = int(M['m10']/M['m00'])
        cy = int(M['01']/M['m00'])
    #blob is blocked or out of camera view = zerodivision
    except ZeroDivisionError:
        #implement chamfer matching here
        print "Needs chamfer matching"



  def pixelToMeterLink1(self,image):
    blue_blob = self.detect_blue(image)
    yellow_blob = self.detect_yellow(image)

    dist = np.sum((blue_blob - yellow_blob)**2)
    return 2.5/np.sqrt(dist)


  def detect_joint2(self,image):
    adj = self.pixelToMeterLink1(image)

    blue_center = adj * self.detect_blue(image)
    yellow_center = adj * self.detect_yellow(image)

    joint_angle2 = np.arctan2(yellow_center[0]-blue_center[0],yellow_center[1]-blue_center[1])
    return joint_angle2

















  # Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)

    im1=cv2.imshow('window1', self.cv_image1)
    cv2.waitKey(1)

    #Get joint angles
    j_angle = self.defineSinusoidalTrajectory()

    # Publish the results
    try:
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))

      self.robot_joint2_pub.publish(j_angle[0])
      self.robot_joint3_pub.publish(j_angle[1])
      self.robot_joint4_pub.publish(j_angle[2])

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
