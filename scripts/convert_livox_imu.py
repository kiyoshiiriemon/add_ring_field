#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu

PUB = rospy.Publisher("livox/imu_corrected", Imu, queue_size=10)

def callback(msg):
    c = Imu()
    c.header = msg.header
    c.linear_acceleration.x = msg.linear_acceleration.x * 9.81
    c.linear_acceleration.y = msg.linear_acceleration.y * 9.81
    c.linear_acceleration.z = msg.linear_acceleration.z * 9.81
    c.angular_velocity.x = msg.angular_velocity.x
    c.angular_velocity.y = msg.angular_velocity.y
    c.angular_velocity.z = msg.angular_velocity.z
    PUB.publish(c)
    print(c)

def main():
    rospy.init_node("imu_converter")
    rospy.Subscriber("livox/imu", Imu, callback)
    rospy.spin()

if __name__ == "__main__":
    main()

