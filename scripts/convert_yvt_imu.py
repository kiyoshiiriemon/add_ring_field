#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu

PUB = rospy.Publisher("hokuyo3d/imu_corrected", Imu, queue_size=10)

def callback(msg):
    c = Imu()
    c.header = msg.header
    c.linear_acceleration.x = msg.linear_acceleration.z
    c.linear_acceleration.y = msg.linear_acceleration.x
    c.linear_acceleration.z = msg.linear_acceleration.y
    c.angular_velocity.x = msg.angular_velocity.z
    c.angular_velocity.y = msg.angular_velocity.x
    c.angular_velocity.z = msg.angular_velocity.y
    PUB.publish(c)
    print(c)

def main():
    rospy.init_node("imu_converter")
    rospy.Subscriber("hokuyo3d/imu", Imu, callback)
    rospy.spin()

if __name__ == "__main__":
    main()

