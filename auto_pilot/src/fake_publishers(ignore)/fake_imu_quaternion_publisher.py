#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import Imu
import math
import tf

def ty():
    pub = rospy.Publisher("android/imu", Imu , queue_size=10)
    rospy.init_node('imu_quaternion_data_publishing_node')
    r = rospy.Rate(10) # 1hz
    while not rospy.is_shutdown():
    	imu_data=Imu()
    	yaw1 = math.radians(40.5)
    	pitch1 = math.radians(40.6)
    	roll1 = math.radians(50.8)#enter values in degrees.they will be converted to radians
    	
    	quaternion = tf.transformations.quaternion_from_euler(roll1, pitch1, yaw1)
        imu_data.orientation.x=quaternion[0]
        imu_data.orientation.y=quaternion[1]
        imu_data.orientation.z=quaternion[2] 
        imu_data.orientation.w=quaternion[3]
        #new_quaternion=[imu_data.orientation.x,  imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w]
        #rpy = tf.transformations.euler_from_quaternion(new_quaternion)
        rpy = tf.transformations.euler_from_quaternion(quaternion)
        yaw = rpy[2]
        pitch = rpy[1]
        roll = rpy[0]

        print("degrees: ")
        print("yaw : "+str(math.degrees(yaw))+"  pitch : "+str(math.degrees(pitch))+"  roll : "+str(math.degrees(roll)))
        print("quaternion: ")
        print("x : "+str(quaternion[0])+"  y : "+str(quaternion[1])+"  z : "+str(quaternion[2])+"  w : "+str(quaternion[3]))
        print("publishing quaternion")
        print("")
        print("")
        print("")
        pub.publish(imu_data)
        r.sleep()

if __name__ == '__main__':
    try:
        ty()
    except rospy.ROSInterruptException:
        pass
