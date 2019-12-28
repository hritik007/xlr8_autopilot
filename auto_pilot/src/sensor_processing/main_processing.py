#!/usr/bin/env python
"""
this is main processing file , this includes'
subscribers to all topic
publishing of master sting
importing of sub files to use their functions
saving all the values globally (v imp!)
"""

#importing required libraries
import rospy
import math
import cv2
import tf
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge() #this object will make transformation between ros and opencv format(both ways)
import numpy as np
import time
import sys

#importing assistant files(larger the number , more the priority)
import gps_and_yaw_processing_0
import ball_processing_1
import arrow_processing_2
import pitch_roll_processing_3
import lidar_processing_4
import delay_operations

#importing messages
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

#master string publisher
pub = rospy.Publisher('master_string_topic', String, queue_size=15)

#tuning parameters
goal_lat  = 28.751385
goal_lon  = 77.119103
min_dist_to_stop=0.0 #rover will stop within 10m radius of the goal coordinates#get rid of this
straight_allowance=2.0 #rover motion will be considered straight within -4 to -4 degrees
rotation_allowance=9.0 #rover will start turning if required rotation is not in -20 to +20 degree
min_dist_for_camera_processing=3.0 #arrow and ball detection will start after this
left_right_searching_threshold=45
manual_mode=0
#note: code is optimized to add marker locations one by one

#setting up global variables
#1) for gps
rover_lat = 0.0
rover_lon = 0.0

#2) for imu
yaw     = 0.0
yaw_360 = 0.0
yaw_360_old = 0.0
yaw_360_increment = 0.0
pitch   = 0.0
roll    = 0.0

#3) for ball
ball_area=0.0
ball_cooridinate=0.0
is_ball_detected=0.0

#for arrow
arrow_direction=0.0

#4) for lidar
temp=LaserScan()
lidar_ranges=temp.ranges

#others
trigger=0
intr=0.0
Bearing=0.0
master_string=String()
master_string="00000"
master_string_old="00000"
node_run_first_time=1
# index 0 : motion type
# index 1 : pwm
# index 2 : pwm
# index 3 : pwm
# index 4 : interrupt

goal_marker_distance=0.0
rover_rotating_angle=0.0

#affinity protocol variables
#a) general rotation
hide_for_affinity_protocol=0
angle_where_roatation_started=0
count_affinity=0
search_count=0
temp_rover_rotating_angle=0
left_right_searching_operaion=1

#b) ball
ball_seen_first_time_flag=0
ball_alignment_already_started_before=0#if ball is detected and rover slowyly starts aligning itself in direction of tha ball, then this becomes 1

#c) arrow
arrow_seen_first_time_flag=0
arrow_rotation_already_started_before=0
angle_where_roatation_started_arrow=0
temp_rover_rotating_angle_arrow=0

"""
is_ball_detected=0.0
arrow_direction=0.0
ball_seen_first_time_flag=0.0
left_right_searching_operaion=0.0
ball_alignment_already_started_before=0.0
arrow_seen_first_time_flag=0.0
arrow_rotation_already_started_before=0.0
angle_where_roatation_started_arrow=0.0
temp_rover_rotating_angle_arrow=0.0
"""

def calculation_and_display_function():
    #globalizing every value found
    global master_string, master_string_old, goal_marker_distance , rover_rotating_angle , trigger, rover_lat,rover_lon,goal_lat,goal_lon
    global intr, min_dist_for_camera_processing , Bearing , is_ball_detected , arrow_direction
    global angle_where_roatation_started, count_affinity , temp_rover_rotating_angle
    global ball_seen_first_time_flag, left_right_searching_operaion , ball_alignment_already_started_before
    global arrow_seen_first_time_flag , arrow_rotation_already_started_before , angle_where_roatation_started_arrow , temp_rover_rotating_angle_arrow
    global yaw_360_increment , yaw_360 , yaw_360_old , left_right_searching_threshold
    global hide_for_affinity_protocol, node_run_first_time
    goal_marker_distance=gps_and_yaw_processing_0.calc_min_distance(rover_lat, rover_lon, goal_lat, goal_lon)
    rover_rotating_angle, Bearing =gps_and_yaw_processing_0.calc_rover_rotating_angle(rover_lat, rover_lon, goal_lat, goal_lon,yaw)

    if(goal_marker_distance <= min_dist_for_camera_processing and search_count<8 and node_run_first_time==0):#affinity circle
        hide_for_affinity_protocol=1
        print("[...Rover is being controlled by AFFINITY CIRCLE PPROTOCOL...]")
        yaw_360_increment=yaw_360-yaw_360_old
        if(yaw_360_increment>120):#+5 to -355
            yaw_360_increment=yaw_360_increment-360
        elif(yaw_360_increment<-120): #-355 to +5
            yaw_360_increment=360+yaw_360_increment

        temp_rover_rotating_angle=temp_rover_rotating_angle+yaw_360_increment
        yaw_360_old=yaw_360

        if(count_affinity==0):
            master_string="12550"
            delay_operations.delay_for_time("avoid_affinity_error")
            #angle_where_roatation_started=rover_rotating_angle#change this to 0 if rover faces problem
            angle_where_roatation_started=rover_rotating_angle
            temp_rover_rotating_angle=0
            count_affinity=1

        print("total rotation from centre   : "+str(temp_rover_rotating_angle))
        print("angle where rotation started : "+str(angle_where_roatation_started))
        ################## in developement ###################################################

        ######################################################################################
        if(count_affinity==1 and left_right_searching_operaion==1):#movement pattern starts here(0 to -70 degree)(anti-cloclwise)
            master_string="31700"
            if((temp_rover_rotating_angle-angle_where_roatation_started)<-left_right_searching_threshold):#if rover reaches -70 degree
                master_string="00000"
                delay_operations.delay_for_time("rotation_safety")
                count_affinity=2

        if(count_affinity==2 and left_right_searching_operaion==1):#-70 degree to +70 degree (clockwise)
            master_string="41700"
            if((temp_rover_rotating_angle-angle_where_roatation_started)>left_right_searching_threshold):#if rover reaches +70 degree
                master_string="00000"
                delay_operations.delay_for_time("rotation_safety")
                count_affinity=3

        if(count_affinity==3 and left_right_searching_operaion==1):#+70 to 0 degree (anticlockwise)
            master_string="31700"
            if((temp_rover_rotating_angle-angle_where_roatation_started)<0):#if rover reaches the point where it started
                master_string="00000"
                delay_operations.delay_for_time("rotation_safety")

                master_string="12000"#move rover straight for 5 seconds
                delay_operations.delay_for_time("marker_forward_fashion")
                count_affinity=1

        """
        if(arrow_direction!=0.0 or left_right_searching_operaion==-2):#if arrow detected then run only this
            temp_rover_rotating_angle_arrow=5000000+yaw_360
            left_right_searching_operaion=-2#no more left right searching operation
            if(arrow_seen_first_time_flag==0):#to give stop and delay only first time when arrow is detected
                master_string="00000"
                arrow_seen_first_time_flag=1
                angle_where_roatation_started_arrow=5000000+yaw_360

            

            if(arrow_direction==-1 or arrow_rotation_already_started_before==-1):#if we detect left arrow
                if(temp_rover_rotating_angle_arrow < (angle_where_roatation_started_arrow-90)):
                    print("rover rotated 90 degrees left :) ")
                    master_string="00000"
                    print("now add next marker location to proceed")
                    goal_lat=input()
                    goal_lon=input()
                arrow_rotation_already_started_before=-1
                master_string="31500"
                print("Left arrow detected ! (so turning arrow 90 degrees left)")
            if(arrow_direction==+1 or arrow_rotation_already_started_before==-2):#if we detect left arrow
                if(temp_rover_rotating_angle_arrow < (angle_where_roatation_started_arrow-90)):
                    print("rover rotated 90 degrees right :) ")
                    master_string="00000"
                    print("now add next marker location to proceed")
                    goal_lat=input()
                    goal_lon=input()
                arrow_rotation_already_started_before=-2
                master_string="41500"
                print("Left arrow detected ! (so turning arrow 90 degrees right)")
        """

        if(is_ball_detected==1.0 or left_right_searching_operaion==-1):#if ball detected then run only this
            left_right_searching_operaion=-1#if ball is detected rover will shut off left and right searching operation #please make this 1 when we move for next marker
            if(ball_seen_first_time_flag==0):#to give stop and delay only first time when ball is detected
                #print("!!Ball is detected for the first time Applying rotation safety delay")
                #print("")
                master_string="00000"
                delay_operations.delay_for_time("rotation_safety")
                ball_seen_first_time_flag=1

            if((ball_cooridinate>=0 and ball_alignment_already_started_before==0 )or (ball_alignment_already_started_before==-1)):#if ball is on right side, then move left side
                print("Ball is detected on right side(moving rover right)")
                if(ball_cooridinate<0):
                    master_string="00000"
                    print("!! NOW ROVER IS FACING THE BALL !! :) ")#marker is now located
                    time.sleep(4)
                    print("now add next marker location to proceed")
                    goal_lat=input("New Latitude  : ")
                    goal_lon=input("New Longitude : ")
                ball_alignment_already_started_before=-1
                master_string="41700"

            elif((ball_cooridinate<0 and ball_alignment_already_started_before==0) or (ball_alignment_already_started_before==-2)):
                print("Ball is detected on left side(moving rover left)")
                if(ball_cooridinate>0):
                    master_string="00000"
                    print("!! NOW ROVER IS FACING THE BALL !! :) ")#marker is now located
                    time.sleep(4)
                    print("now add next marker location to proceed")
                    goal_lat=input("New Latitude  : ")
                    goal_lon=input("New Longitude : ")
                ball_alignment_already_started_before=-2
                master_string="31700"
            else:
                print("now ball is detected, next thing to do is defined :) ")

    elif(lidar_processing_4.check_if_obstacle_detected() == True and False and node_run_first_time==0):#if lidar detects obstacle
        print("[...Rover is being controlled by LIDAR...]")
        master_string=lidar_processing_3.master_string_generator()
    elif(pitch_roll_processing_3.check_if_extreme_pitch_or_roll_detected() == True and False and node_run_first_time==0):#wrong pitch+roll values sensed
        print("[...Rover is being controlled by PITCH AND ROLL...]")
        master_string=pitch_roll_processing_2.master_string_generator()
    else:#gps and orientation(yaw)
        print("[...Rover is being controlled by GPS and yaw...]")
        node_run_first_time=0
        master_string_old=master_string
        master_string,trigger=gps_and_yaw_processing_0.master_string_generator(goal_marker_distance, rover_rotating_angle,trigger,min_dist_to_stop,straight_allowance,rotation_allowance)
        if(master_string[0]!=master_string_old[0]):
            delay_operations.delay_for_time("rotation_safety")

    #move below lines in listener later.
    print("")
    if(hide_for_affinity_protocol==0):
        print("exteme pitch/roll status  : 0.0")
        print("obstacle detection status : 0.0")
    print("gps bearing               : "+str(Bearing))
    print("yaw (-180 to +180) format : "+str(yaw)+" degrees")
    print("yaw (0 to 360) format     : " + str(yaw_360))
    print("goal  (lat , lon)         : ( "+str(goal_lat)+" , "+str(goal_lon)+" )")
    print("rover (lat , lon)         : ( "+str(rover_lat)+" , "+str(rover_lon)+" )")
    print("rover rotation required   : "+str(rover_rotating_angle)+" degrees")
    print("Distance to goal          : "+str(goal_marker_distance)+"  meters")
    print("master string             : "+str(master_string))
    print("...............................................................................")

#here callbacks are defined just to assign values in global variables
#all processing will be done in separate python file and then imported
def callback_gps(gps_data):
    global rover_lat,rover_lon,count
    rover_lat = gps_data.latitude  #present latitude
    rover_lon = gps_data.longitude #present longitude

def callback_lidar(laser_scan_data):
    #global lidar_ranges
    lidar_ranges_0_to_359=laser_scan_data.ranges
    lidar_ranges_minus_40_to_plus_39=lidar_processing_4.ranges_trimmer(lidar_ranges_0_to_359)#-40 to +39
    
    ###################normalizer function###############################
    lidar_ranges_minus_40_to_minus_1 = lidar_ranges_minus_40_to_plus_39[0:40]#-40 to -1 degree  (total 40 values)
    lidar_ranges_0_to_39 = lidar_ranges_minus_40_to_plus_39[40:80]#0 to 39 degree (total 40 values)

    base_minus_40_to_minus_1=np.zeros(40)
    base_0_to_40=np.zeros(40)
    
    for r in range(40):#0 to 39 == -40 to -1 in range
        r2=40-r#40 , 39 , 38 , ..... , 2, 1
        r2=r2*-1#-40 , -39 , -38 , ...... ,-2 , -1
        base_minus_40_to_minus_1[r]=math.cos(math.radians(r2))*lidar_ranges_minus_40_to_minus_1[r]

    for r in range(40):#0 to 39 == 0 to 39 in range
        base_0_to_40[r]=math.cos(math.radians(r))*lidar_ranges_0_to_39[r]

    normalized_ranges_minus_40_to_plus_39=np.concatenate((base_minus_40_to_minus_1,base_0_to_40 ), axis=0)
    #####################################################

    ###################3temporary printing function (for debugging)############################
    """
    temp2=-40
    for rr in  normalized_ranges_minus_40_to_plus_39:
        print(str(temp2)+" degrees : range = "+str(rr)+" meters")
        temp2=temp2+1
    print("")
    """
    
    ############################################################

    ##########################10 degrees average taker###########################
    avreage_normalized_ranges=np.zeros(8)
    for i in range(8):
        for j in range(10):
            avreage_normalized_ranges[i]=avreage_normalized_ranges[i]+normalized_ranges_minus_40_to_plus_39[(i*10)+j]
    avreage_normalized_ranges=avreage_normalized_ranges/10
    ##############################################################################3

    ######################################################################
    """
    for i in range(8):
        print(avreage_normalized_ranges[i])
    print("")
    """
    #############################################################################

def callback_ball(ball_data):
    global ball_area , ball_cooridinate , is_ball_detected
    ball_area=ball_data.x
    ball_cooridinate=ball_data.y
    if(goal_marker_distance <= min_dist_for_camera_processing):
        is_ball_detected=ball_data.z
    else:
        is_ball_detected=0.0

def callback_arrow(arrow_dir):
    global arrow_direction
    if(goal_marker_distance <= min_dist_for_camera_processing):
        arrow_direction=arrow_dir.data
    else:
        arrow_direction=0

def callback_imu(imu_data):
    global yaw,pitch,roll, yaw_360
    quaternion=[imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w]
    yaw,pitch,roll = gps_and_yaw_processing_0.quaternion_to_ypr(quaternion)

    yaw_360=yaw+180
    yaw_360=360-yaw_360
    
    #samsung phone tested
    yaw=yaw*-1#samsung gives opposite sign values
    if(yaw>180):
        yaw=yaw-360

    calculation_and_display_function()
    pub.publish(master_string)

    
def listener():
    rospy.init_node('main_processing_node', anonymous=False)
    
    rospy.Subscriber("android/fix", NavSatFix , callback_gps)
    #rospy.Subscriber("scan", LaserScan , callback_lidar)
    rospy.Subscriber("ball_data_topic", Point , callback_ball)
    #rospy.Subscriber("arrow_direction_topic", Int16 , callback_arrow)
    rospy.Subscriber("android/imu", Imu , callback_imu)#it is new

    #code at this line ------------ willwork only once ! wtf!!
    rospy.spin()

if __name__ == '__main__':
    print("Activating Autopilot Mode")
    listener()
