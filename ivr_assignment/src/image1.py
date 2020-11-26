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
import math


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera1 to a topic named image_topic1
    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)


    # set up publisher to send joint angles to the robot
    self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command",Float64,queue_size=10)
    self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command",Float64,queue_size=10)
    self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command",Float64,queue_size=10)
    self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command",Float64,queue_size=10)

    # sets up publisher of estimated joints via computer vision from camera 1
    self.est1_joint2_pub = rospy.Publisher("/robot/joint2_estimated",Float64,queue_size=10)
    self.est1_joint3_pub = rospy.Publisher("/robot/joint3_estimated",Float64,queue_size=10)
    self.est1_joint4_pub = rospy.Publisher("/robot/joint4_estimated",Float64,queue_size=10)

    # sets up subscribers to get zx coords of blobs from camera 2
    self.blue_z_sub = rospy.Subscriber("blue_z_coord",Float64,self.get_blue_z)
    self.blue_x_sub = rospy.Subscriber("blue_x_coord",Float64,self.get_blue_x)

    self.green_z_sub = rospy.Subscriber("green_z_coord",Float64,self.get_green_z)
    self.green_x_sub = rospy.Subscriber("green_x_coord",Float64,self.get_green_x)

    self.red_z_sub = rospy.Subscriber("red_z_coord",Float64,self.get_red_z)
    self.red_x_sub = rospy.Subscriber("red_x_coord",Float64,self.get_red_x)


    # global variables of camera 2 coords
    self.blueC2_z = 0.0
    self.blueC2_x = 0.0
    self.greenC2_z = 0.0
    self.greenC2_x = 0.0
    self.redC2_z = 0.0
    self.redC2_x = 0.0

    # pixel to meter conversion
    self.pToM = 0.0;

    # negative/positive







    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()

    #start time for robot
    self.time_trajectory = rospy.get_time()


  def calculateDistance(self,x1,x2,y1,y2,z1,z2):
    point_1 = np.array((x1,y1,z1))
    point_2 = np.array((x2,y2,z2))
    return np.linalg.norm(point_1 - point_2)


  def defineSinusoidalTrajectory(self):
    #get current time
    cur_time = np.array([rospy.get_time() - self.time_trajectory])
    print("Time:",cur_time)
    joint2_trajectory = float(np.pi/2*np.sin((np.pi/15)*cur_time))
    joint3_trajectory = float(np.pi/2*np.sin((np.pi/18)*cur_time))
    joint4_trajectory = float(np.pi/2*np.sin((np.pi/20)*cur_time))
    return np.array([joint2_trajectory,joint3_trajectory,joint4_trajectory])

  def detect_red(self,image):
    mask = cv2.inRange(image,(0,0,100),(0,0,255))
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.dilate(mask,kernel,iterations=3)
    try:
      M = cv2.moments(mask)
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
    except ZeroDivisionError:
      #checks if red is blocked by green and blue or orange blob
      g = self.detect_green(image)
      #if is blocked by blue/green then green is blocked as well
      #hence detect_green should return the negative flag
      if(g[1] == -100):
          cx = -100
          cy = -100

      else:
          #gives it a different flag when blocked by orange blob
          cx = -200
          cy = -200

    return np.array([cx,cy])

  def detect_green(self,image):
    mask = cv2.inRange(image,(0,100,0),(0,255,0))
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.dilate(mask,kernel,iterations=3)
    M = cv2.moments(mask)
    try:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
    except ZeroDivisionError:
      #green blob is being blocked by blue blob
      #-100s are a flag to tell function to recalculate
      #cx,cy
      cx = -100
      cy = -100
    return np.array([cx,cy])



  def detect_blue(self,image):
    mask = cv2.inRange(image,(100,0,0),(255,0,0))
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.dilate(mask,kernel,iterations=3)
    M = cv2.moments(mask)
    try:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
    except ZeroDivisionError:
      #minus pixel values are a flag to tell
      #joint detector to recalculate cx,cy
      cx = -100
      cy = -100

    return np.array([cx,cy])


  def detect_yellow(self,image):
    mask = cv2.inRange(image,(0,100,100),(0,255,255))
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.dilate(mask,kernel,iterations=3)

    M = cv2.moments(mask)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    return np.array([cx,cy])

  # used in joint calculation for exception of orange blob
  # blocking red blob
  def detect_orange(self,image):
    mask = cv2.inRange(image,(5,50,50),(15,255,255))
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.dilate(mask,kernel,iterations=3)

    M = cv2.moments(mask)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    return np.array([cx,cy])





  def pixelToMeter(self,image):
    blue_blob = self.detect_blue(image)
    yellow_blob = self.detect_yellow(image)

    dist = np.sum((yellow_blob - blue_blob)**2)
    return 2.5/np.sqrt(dist)


  def detect_joint2(self,image):
    p = self.pixelToMeter(image)
    green_blob = self.detect_green(image)
    blue_blob = self.detect_blue(image)

    #checks for ZeroDivisionError flags
    if (blue_blob[0] == -100):
      print("robot pointing at camera")
      blue_blob = green_blob

    if (green_blob[0] == -100):
      #green is being blocked by blue so z and y
      #coords (in terms of base frame at yellow) are the same
      print("robot pointing away from camera")
      green_blob = blue_blob

    # gets average of z coords for 3d space
    green_blob_z = math.floor((green_blob[1]+self.greenC2_z)/2)
    blue_blob_z = math.floor((blue_blob[1]+self.blueC2_z)/2)

    hyp = p*self.calculateDistance(self.greenC2_x,self.blueC2_x,green_blob[0],blue_blob[0],green_blob_z,blue_blob_z)
    adj = p*self.calculateDistance(self.greenC2_x,self.greenC2_x,green_blob[0],green_blob[0],green_blob_z,blue_blob_z)

    ratio = adj/hyp
    #print("Adj",adj)
    #print("Hyp",hyp)
    #print("Ratio",ratio)
    j2_angle = np.arccos(ratio)
    return j2_angle


  def detect_joint3(self,image,j2_angle):
    p = self.pixelToMeter(image)
    green_blob = self.detect_green(image)
    blue_blob = self.detect_blue(image)

    #checks for ZeroDivisionError flags
    if (blue_blob[0] == -100):
      print("robot pointing at camera")
      blue_blob = green_blob

    if (green_blob[0] == -100):
      #green is being blocked by blue so z and y
      #coords (in terms of base frame at yellow) are the same
      green_blob = blue_blob

    green_blob_z = math.floor((green_blob[1]+self.greenC2_z)/2)
    blue_blob_z = math.floor((blue_blob[1]+self.blueC2_z)/2)


    hyp = p*self.calculateDistance(self.greenC2_x,self.blueC2_x,green_blob[0],blue_blob[0],blue_blob_z,blue_blob_z)
    adj = p*self.calculateDistance(self.blueC2_x,self.blueC2_x,green_blob[0],blue_blob[0],blue_blob_z,blue_blob_z)

    ratio = adj/hyp
    j3_angle = np.arccos(ratio)
    return j3_angle


  def detect_joint4(self,image,j2_angle,j3_angle):
    p = self.pixelToMeter(image)
    blue_blob = self.detect_blue(image)
    red_blob = self.detect_red(image)
    green_blob = self.detect_green(image)

    #checks for ZeroDivisionError flags
    if (green_blob[0] == -100):
      #green is being blocked by blue so z and y
      #coords (in terms of base frame at yellow) are the same
      green_blob = blue_blob

    if (red_blob[0] == -100):
      #blue is being blocked by greenn which is being blocked by blue so z and y
      #coords (in terms of base frame at yellow) are the same for all blobs
      red_blob = blue_blob

    red_blob_z = math.floor((red_blob[1]+self.redC2_z)/2)
    green_blob_z = math.floor((green_blob[1]+self.greenC2_z)/2)
    blue_blob_z = math.floor((blue_blob[1]+self.blueC2_z)/2)

    hyp = p*self.calculateDistance(self.redC2_x,self.greenC2_x,red_blob[0],green_blob[0],red_blob_z,green_blob_z)
    adj = p*self.calculateDistance(self.redC2_x,self.redC2_x,red_blob[0],red_blob[0],red_blob_z,green_blob_z)

    ratio = adj/hyp
    j4_angle = np.arccos(ratio)
    return j4_angle



  # functions to get coord data from camera 2
  # x,z axes of the base frame
  def get_blue_z(self,data):
    self.blueC2_z = data.data

  def get_blue_x(self,data):
    self.blueC2_x = data.data

  def get_green_z(self,data):
    self.greenC2_z = data.data

  def get_green_x(self,data):
    self.greenC2_x = data.data

  def get_red_z(self,data):
    self.redC2_z = data.data

  def get_red_x(self,data):
    self.redC2_x = data.data





  # Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Uncomment if you want to save the image


    im1=cv2.imshow('window1', self.cv_image1)

    cv2.waitKey(1)




    # Get joint angles for trajectory
    j_angle = self.defineSinusoidalTrajectory()



    # Publish the results
    try:
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))


      self.robot_joint2_pub.publish(j_angle[0])
      self.robot_joint3_pub.publish(j_angle[1])
      self.robot_joint4_pub.publish(j_angle[2])


      est_j2 = self.detect_joint2(self.cv_image1)
      est_j3 = self.detect_joint3(self.cv_image1,est_j2)
      est_j4 = self.detect_joint4(self.cv_image1,est_j2,est_j3)

      self.est1_joint2_pub.publish(est_j2)
      self.est1_joint3_pub.publish(est_j3)
      self.est1_joint4_pub.publish(est_j4)
      print("J2 Actual Angle:",j_angle[0]," Estimated Angle:",est_j2)
      print("J3 Actual Angle:",j_angle[1]," Estimated Angle:",est_j3)
      print("J4 Actual Angle:",j_angle[2]," Estimated Angle:",est_j4)


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
