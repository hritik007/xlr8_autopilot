#!/usr/bin/env python
import rospy
import cv2
from geometry_msgs.msg import Point
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
import numpy as np
import time

bridge = CvBridge() #this object will make transformation between ros and opencv format(both ways)
ball_data=Point()
ball_data.x=0.0
ball_data.y=0.0
ball_data.z=0.0

ball_area_old=0.0
ball_cooridinate_old=0.0










#talker.py source
def ty(ball_data):#a publisher function to publish x coordinate of the ball (cx)
    pub = rospy.Publisher('ball_data_topic', Point , queue_size=10)#here ball_data is message type
    r = rospy.Rate(10) # 1hz
    print("Ball area       : "+str(ball_data.x))
    print("Ball coorinates : "+str(ball_data.y))
    print("Ball detected   : "+str(ball_data.z))
    print("")
    pub.publish(ball_data)

#ball tracking.py source
def filter_color(rgb_image, lower_bound_color, upper_bound_color):
    #convert the image into the HSV color space
    hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
    #cv2.imshow("hsv image",hsv_image)

    #define a mask using the lower and upper bounds of the yellow color 
    mask = cv2.inRange(hsv_image, lower_bound_color, upper_bound_color)

    return mask

def getContours(binary_image):     
    #_, contours, hierarchy = cv2.findContours(binary_image, 
    #                                          cv2.RETR_TREE, 
    #                                           cv2.CHAIN_APPROX_SIMPLE)
    _, contours, hierarchy = cv2.findContours(binary_image.copy(), 
                                            cv2.RETR_EXTERNAL,
	                                        cv2.CHAIN_APPROX_SIMPLE)
    return contours

def draw_ball_contour(binary_image, rgb_image, contours):
    black_image = np.zeros([binary_image.shape[0], binary_image.shape[1],3],'uint8')
    
    for c in contours:
        area = cv2.contourArea(c)
        perimeter= cv2.arcLength(c, True)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        if (area>300):#play with this in inferno lab
            cv2.drawContours(rgb_image, [c], -1, (150,250,150), 1)
            cv2.drawContours(black_image, [c], -1, (150,250,150), 1)
            cx,cy = get_contour_center(c)
            ball_cooridinate=cx-318
            #global ball_data, 
            ball_data.x=area
            ball_data.y=ball_cooridinate

            cv2.circle(rgb_image, (cx,cy),(int)(radius),(0,0,255),1)
            cv2.circle(black_image, (cx,cy),(int)(radius),(0,0,255),1)
            cv2.circle(black_image, (cx,cy),5,(150,150,255),-1)
            #print ("Area: {}, Perimeter: {}".format(area, perimeter))
            #print("x: {}, y: {}".format(cx,cy) )
    #print ("number of contours: {}".format(len(contours)))
    cv2.imshow("Ball Detection",rgb_image)
    #cv2.imshow("Black Image Contours",black_image)
    return ball_data

def get_contour_center(contour):
    M = cv2.moments(contour)
    cx=-1
    cy=-1
    if (M['m00']!=0):
        cx= int(M['m10']/M['m00'])
        cy= int(M['m01']/M['m00'])
    return cx, cy

def detect_ball_in_a_frame(image_frame):
    yellowLower =(30, 100, 50)
    yellowUpper = (60, 255, 255)
    rgb_image = image_frame
    binary_image_mask = filter_color(rgb_image, yellowLower, yellowUpper)
    contours = getContours(binary_image_mask)
    ball_data=draw_ball_contour(binary_image_mask, rgb_image,contours)
    return ball_data




















#image pub sub original
def image_callback(ros_image):#working on 30fps
  #print 'got an image'

  global bridge
  #convert ros_image(imgmsg) into an opencv-compatible image
  try:
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8") #bgr8 is a 8 bit encoded format
  except CvBridgeError as e:
      print(e)

  #after above line, we can do enaything that opencv can do, for example see below
  #from now on, you can work exactly like with opencv

  #applying gamma correction
  lookUpTable = np.empty((1,256), np.uint8)
  gamma=1 #set gamma value here :) (lesser for brighter,more for darker)
  for i in range(256):
    lookUpTable[0,i] = np.clip(pow(i / 255.0, gamma) * 255.0, 0, 255)
  cv_image = cv2.LUT(cv_image, lookUpTable)

  ball_data=detect_ball_in_a_frame(cv_image)

  global ball_area_old,ball_cooridinate_old

  if((ball_cooridinate_old==ball_data.y) and (ball_area_old==ball_data.x)):
    ball_data.z=0.0
  else:
    ball_data.z=1.0

  ball_area_old=ball_data.x
  ball_cooridinate_old=ball_data.y

  ty(ball_data)
  cv2.waitKey(3)

  




def listener():
  rospy.init_node('ball_preprocessing_node', anonymous=False) #initialising a node named 'image_converter'
  image_sub = rospy.Subscriber("/usb_cam/image_raw",Image, image_callback) #creating a subscriber to image
  ##Image is a data type defined in ros,,
  #whenever we receive an image th. /usb_cam/image_raw ,etc topic , we are going to execute the image callback
  #image callback has the image in ros format and we want to transform in form of open cv
  #so we will use the bridge
  #then based on the ros image collected from the topic, we are going to create cv_image for open cv image using imgmsg_to_cv2
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()






if __name__ == '__main__':
    listener()
