#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import math
import numpy as np
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class kinematics:

  # Defines publisher and subscriber
  def __init__(self):
    self.cv_x_estimates = []
    self.cv_y_estimates = []
    self.cv_z_estimates = []

    self.fk_estimates = []
    self.cv_estimates = []

    self.bridge = CvBridge()

    # initialize the node named image_processing
    rospy.init_node('kinematics', anonymous=True)
    # initialize a publisher to send predicted x y and z coordinates of end effector
    self.kin_x = rospy.Publisher("kinematics_x",Float64, queue_size = 10)
    self.kin_y = rospy.Publisher("kinematics_y",Float64, queue_size = 10)
    self.kin_z = rospy.Publisher("kinematics_z",Float64, queue_size = 10)

    #subscribers for visual estimates
    self.cv_estimate_x = rospy.Subscriber("cv_estimate_x",Float64,self.cvx_callback)
    self.cv_estimate_y = rospy.Subscriber("cv_estimate_y",Float64,self.cvy_callback)
    self.cv_estimate_z = rospy.Subscriber("cv_estimate_z",Float64,self.cvz_callback)

    #set up publisher to send joint angles to the robot
    self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command",Float64,queue_size=10)
    self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command",Float64,queue_size=10)
    self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command",Float64,queue_size=10)
    self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command",Float64,queue_size=10)


    #initialize subscribera to recieve messages from a topics named /robot/camera1/image_raw and use joint_test function to recieve data
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.get_image_data_2)
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.joint_test)



    #start time for robot
    self.time_trajectory = rospy.get_time()


    #self.joint_test()
    #self.cv_estimates = [[self.cv_x_estimates[i],self.cv_y_estimates[i],self.cv_z_estimates[i]] for i in range(len(self.cv_x_estimates))]
    self.showResults()




  def returnFinalTMat(self,theta1,theta2,theta3,theta4):
    m1 = self.calculateTMatFromDHParams(-np.pi/2,0,2.5,theta1)
    m2 = self.calculateTMatFromDHParams(np.pi/2,0,0,theta2)
    m3 = self.calculateTMatFromDHParams(-np.pi/2,3.5,0,theta3)
    m4 = self.calculateTMatFromDHParams(np.pi/2,0,3,theta4)
    final = np.matmul(m1,np.matmul(m2,m3,m4))
    return final

  def calculateForwardKinematics(self,theta1,theta2,theta3,theta4):
    m1 = self.calculateTMatFromDHParams(-np.pi/2,0,2.5,theta1)
    m2 = self.calculateTMatFromDHParams(np.pi/2,0,0,theta2)
    m3 = self.calculateTMatFromDHParams(-np.pi/2,3.5,0,theta3)
    m4 = self.calculateTMatFromDHParams(np.pi/2,0,3,theta4)
    final = np.matmul(m1,np.matmul(m2,m3,m4))
    return final[:,3][:3]

  def calculateTMatFromDHParams(self, alpha, a, d, theta):
    t = [np.cos(theta),-np.sin(theta)*np.cos(alpha),np.sin(theta)*np.sin(alpha), a*np.cos(theta),
         np.sin(theta),np.cos(theta)*np.cos(alpha),-np.cos(theta)*np.sin(alpha), a*np.sin(theta),
         0, np.sin(alpha), np.cos(alpha), d,
         0,0,0,1]
    t = np.array(t).reshape((4,4))
    return t

  def getAllTransformMatrices(self,theta1,theta2,theta3,theta4):
    m1 = self.calculateTMatFromDHParams(-np.pi/2,0,2.5,theta1)
    m2 = self.calculateTMatFromDHParams(np.pi/2,0,0,theta2)
    m3 = self.calculateTMatFromDHParams(-np.pi/2,3.5,0,theta3)
    m4 = self.calculateTMatFromDHParams(np.pi/2,0,3,theta4)
    frames = [m1,m2,m3,m4]
    t_mats = []
    t_mats.append(np.eye(shape=((m1.shape))))#0-1
    t_mats.append(np.matmul(m1,m2))#0-2
    t_mats.append(np.matmul(t_mats[1],m3))#0-3
    t_mats.append(np.matmul(t_mats[2],m4))#0-4
    return t_mats

  def calculateJacobian(self,a,b,c,d):
    m11 = np.cos(d)*(np.cos(a)*np.cos(b)*np.cos(c)-np.sin(a)*np.sin(c))-np.cos(a)*np.sin(b)*np.sin(d)
    m12 = -np.cos(c)*np.sin(a)-np.cos(a)*np.cos(b)*np.sin(c)
    m13 =  np.cos(a)*np.cos(d)*np.sin(b)+np.sin(d)*(np.cos(a)*np.cos(b)*np.cos(c)-np.sin(a)*np.sin(c))
    m14 =  3.5*np.cos(a)*np.sin(b)+3*(-np.cos(c)*np.sin(a)-np.cos(a)*np.cos(b)*np.sin(c))

    m21 = np.cos(d)*(np.cos(b)*np.cos(c)*np.sin(a)+np.cos(a)*np.sin(c))-np.sin(a)*np.sin(b)*np.sin(d)
    m22 = np.cos(a)*np.cos(c)-np.cos(b)*np.sin(a)*np.sin(c)
    m23 = np.cos(d)*np.sin(a)*np.sin(b)+np.sin(d)*(np.cos(b)*np.cos(c)*np.sin(a)+np.cos(a)*np.sin(c))
    m24 = 3.5*np.sin(a)*np.sin(b)+3*(np.cos(a)*np.cos(c)-np.cos(b)*np.sin(a)*np.sin(c))

    m31 = -np.cos(c)*np.cos(d)*np.sin(b)-np.cos(b)*np.sin(d)
    m32 = np.sin(b)*np.sin(c)
    m33 = np.cos(b)*np.cos(d)-np.cos(c)*np.sin(b)*np.sin(d)
    m34 = 3.5*np.cos(b)+3*np.sin(b)*np.sin(c)+2.5

    m41 = 0
    m42 = 0
    m43 = 0
    m44 = 0

    jacobian = [[m11,m12,m13,m14],
                [m21,m22,m23,m24],
                [m31,m32,m33,m34],
                [m41,m42,m43,m44]]
    return jacobian

  def detect_red(self,image):
    mask = cv2.inRange(image,(0,0,100),(0,0,255))
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.dilate(mask,kernel,iterations=3)
    try:
      M = cv2.moments(mask)
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
    except ZeroDivisionError:
    #checks if red is blocked by green and blue
      g = self.detect_green(image)
      cx = g[0]
      cy = g[1]
    return np.array([cx,cy])

  def detect_yellow(self,image):
    mask = cv2.inRange(image,(0,100,100),(0,255,255))
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.dilate(mask,kernel,iterations=3)

    M = cv2.moments(mask)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    return np.array([cx,cy])

  def detect_green(self,image):
    mask = cv2.inRange(image,(0,100,0),(0,255,0))
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.dilate(mask,kernel,iterations=3)
    M = cv2.moments(mask)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    return np.array([cx,cy])


  def pixelToMeter(self,image):
    red_blob = self.detect_red(image)
    green_blob = self.detect_green(image)

    dist = np.sum((green_blob -red_blob)**2)
    return 3/np.sqrt(dist)


  def detect_end_effector(self,r_xz,r_yz,yel_xz,yel_yz,image):
    yel_z_coord = math.floor((yel_xz[1]+yel_yz[1])/2)

    z_coord = math.floor((r_xz[1]+r_yz[1])/2)
    p = self.pixelToMeter(image)
    z_coord = p*(yel_z_coord - z_coord)
    x_coord = p*(yel_xz[0] - r_xz[0])
    y_coord = p*(yel_yz[0] - r_yz[0])

    return np.array([x_coord,y_coord,z_coord])



  def get_image_data_2(self,data):

    self.cv_image2 = self.bridge.imgmsg_to_cv2(data,"bgr8")

  # Recieve data from camera 1, process it, and publish
  def joint_test(self,data):

    #get image data
    self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")






    joint_positions = [[-1.5707963267948966, 0.5235987755982988, -0.7853981633974483, 0.5235987755982988],
                       [1.5707963267948966, 0.7853981633974483, -0.5235987755982988, -0.7853981633974483],
                       [0.5235987755982988, 1.5707963267948966, 0.5235987755982988, -0.7853981633974483],
                       [-0.7853981633974483, -0.7853981633974483, -1.5707963267948966, 1.5707963267948966],
                       [1.5707963267948966, 1.5707963267948966, 1.5707963267948966, -0.7853981633974483],
                       [1.5707963267948966, -1.5707963267948966, -0.5235987755982988, -0.5235987755982988],
                       [1.5707963267948966, -0.7853981633974483, -0.7853981633974483, -0.5235987755982988],
                       [-0.5235987755982988, 0.5235987755982988, 0.7853981633974483, -0.7853981633974483],
                       [0.5235987755982988, 0.5235987755982988, -0.7853981633974483, 0.5235987755982988],
                       [0.5235987755982988, 0.7853981633974483, -0.7853981633974483, 0.5235987755982988]]

    for joint in joint_positions:
      self.robot_joint1_pub.publish(joint[0])
      self.robot_joint2_pub.publish(joint[1])
      self.robot_joint3_pub.publish(joint[2])
      self.robot_joint4_pub.publish(joint[3])

      #calculates end effector position using CV
      red_yz = self.detect_red(self.cv_image1)
      red_xz = self.detect_red(self.cv_image2)

      #yellow being the base frame we calculate with respect to
      yellow_yz = self.detect_yellow(self.cv_image1)
      yellow_xz = self.detect_yellow(self.cv_image2)

      ee_coords = self.detect_end_effector(red_xz,red_yz,yellow_xz,yellow_yz,self.cv_image1)
      print("CV Estimate: ",ee_coords)

      FK_estimate = self.calculateForwardKinematics(joint[0],joint[1],joint[2],joint[3])
      print("FK Estimate: ",FK_estimate)

      #get jacobian
      jacob = self.calculateJacobian(joint[0],joint[1],joint[2],joint[3])
      print("Jacobian: ",jacob)
      self.fk_estimates.append(FK_estimate)

      time.sleep(4)

  #depreciated function for showing results
  def showResults(self):
    for i in range(len(self.fk_estimates)):
      print("FK estimate: ",self.fk_estimates[i])

  def cvx_callback(self,data):
    self.cv_x_estimates.append(data)

  def cvy_callback(self,data):
    self.cv_y_estimates.append(data)

  def cvz_callback(self,data):
    self.cv_z_estimates.append(data)




# call the class
def main(args):
  k = kinematics()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
