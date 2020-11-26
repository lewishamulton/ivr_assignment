#!/usr/bin/env python3


import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


# Publish data
def move():
  rospy.init_node('target_pos_cmd', anonymous=True)
  rate = rospy.Rate(30) # 30hz
  # initialize a publisher to send joints' angular position to the robot
  robot_joint1_pub = rospy.Publisher("/target/x_position_controller/command", Float64, queue_size=10)
  robot_joint2_pub = rospy.Publisher("/target/y_position_controller/command", Float64, queue_size=10)
  robot_joint3_pub = rospy.Publisher("/target/z_position_controller/command", Float64, queue_size=10)
  robot_joint4_pub = rospy.Publisher("/target2/x2_position_controller/command", Float64, queue_size=10)
  robot_joint5_pub = rospy.Publisher("/target2/y2_position_controller/command", Float64, queue_size=10)
  robot_joint6_pub = rospy.Publisher("/target2/z2_position_controller/command", Float64, queue_size=10)
  t0 = rospy.get_time()

  while not rospy.is_shutdown():

    cur_time = np.array([rospy.get_time()])-t0
    #y_d = float(6 + np.absolute(1.5* np.sin(cur_time * np.pi/100)))


    x_d = 2.5* np.cos(cur_time * np.pi/15)
    y_d = 2.5* np.sin(cur_time * np.pi/15)
    z_d = 1* np.sin(cur_time * np.pi/15)
    joint1=Float64()
    joint1.data= 0.5 + x_d
    joint2=Float64()
    joint2.data= 0 + y_d
    joint3=Float64()
    joint3.data= 7 + z_d
    robot_joint1_pub.publish(joint1)
    robot_joint2_pub.publish(joint2)
    robot_joint3_pub.publish(joint3)
    x_d = 2+ 2* np.cos(cur_time * np.pi/15)
    y_d = 2.5+ 1.5* np.sin(cur_time * np.pi/15)
    joint4=Float64()
    joint4.data=  x_d
    joint5=Float64()
    joint5.data=  y_d
    joint6=Float64()
    joint6.data= 7.5
    robot_joint4_pub.publish(joint4)
    robot_joint5_pub.publish(joint5)
    robot_joint6_pub.publish(joint6)
    rate.sleep()



def get_image_data(data):
    # Recieve the image
    try:
      cv_image1 = bridge.imgmsg_to_cv2(data, "bgr8")

      cnts,thresh = detect_orange(cv_image1)
      print(cnts)
      cnts = imutils.grab_contours(cnts)
      crop_image_samples(cnts,thresh)


    except CvBridgeError as e:
      print(e)


# returns contours of orange masked parts of image
# and thresholded image
def detect_orange(image):

    mask = cv2.inRange(image,(5,50,50),(15,255,255))
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.dilate(mask,kernel,iterations=3)
    ret,thresh = cv2.threshold(mask,127,255,0)
    contours,hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    return contours,thresh

def crop_image_samples(cnts,thresh):
    num = 0
    for c in cnts:
        M = cv2.moments(c)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        center = np.array([cx,cy])
        #creates a 20 by 20 cropped image of contour as sample
        sample = thresh[c[1]-20:c[1]+20,c[0]-20:c[0]+20]
        cv2.imwrite("sample"+num+".jpg",sample)
        num += 1

# run the code if the node is called
if __name__ == '__main__':
  try:
    bridge = CvBridge()
    #get image data
    image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,get_image_data)
    move()



  except rospy.ROSInterruptException:
    pass
