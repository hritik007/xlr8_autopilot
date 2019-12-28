#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import NavSatFix

def ty():
    pub = rospy.Publisher("android/fix", NavSatFix , queue_size=10)
    rospy.init_node('GPS_data_publishing_node')
    r = rospy.Rate(4) # 1hz
    while not rospy.is_shutdown():
        gps_data=NavSatFix()
        
        gps_data.latitude=28.751436
        gps_data.longitude=77.119252

        pub.publish(gps_data)

        print(gps_data.latitude)
        print(gps_data.longitude)
        print("")
        
        r.sleep()

if __name__ == '__main__':
    try:
        ty()
    except rospy.ROSInterruptException:
        pass
