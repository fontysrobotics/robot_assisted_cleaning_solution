#!/usr/bin/env python
import rospy
import tf
from tf import transformations
import math

from nav_msgs.msg import Odometry
from assisted_cleaning_solution.msg import Sensor

class odom_publisher():
    def __init__(self):
        rospy.init_node('odometry_publisher')
        self.odom_publisher = rospy.Publisher('odom', Odometry, queue_size=10)
        self.odom_broadcaster = tf.TransformBroadcaster()

        self.speed_subscriber = rospy.Subscriber('sensor', Sensor, self.speed_CB)
        self.timer = rospy.Timer(rospy.Duration(1), self.timer_callback)

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.vx = 0.0 # encoder speed
        self.vy = 0.0
        self.vth = 0.0 # ((right_speed - left_speed)/lengthWheelBase)
 
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

        self.r = rospy.Rate(1)

    def timer_callback(self, timer):
        self.execute()

    def speed_CB(self, speed):
        self.vx = speed.vx
        self.vth = speed.vth

    def execute(self):
        self.current_time = rospy.Time.now()
        
        self.calculate_position()
        
        odom_quat = transformations.quaternion_from_euler(0, 0, self.th)

        self.odom_broadcaster.sendTransform((self.x, self.y, 0), (odom_quat[0], odom_quat[1], odom_quat[2], odom_quat[3]), self.current_time, "base_footprint", "odom")

        self.odom_message(odom_quat)

        self.last_time = self.current_time
        self.r.sleep()

    def calculate_position(self):
        dt = (self.current_time - self.last_time).to_sec()
        delta_x = (self.vx * math.cos(self.th) - self.vy * math.sin(self.th)) * dt
        delta_y = (self.vx * math.sin(self.th) + self.vy * math.cos(self.th)) * dt
        delta_th = self.vth * dt

        self.x = delta_x + self.x
        self.y = delta_y + self.y
        self.th = delta_th + self.th

    def odom_message(self, odom_quat):
        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = "odom"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = odom_quat[0]
        odom.pose.pose.orientation.y = odom_quat[1]
        odom.pose.pose.orientation.z = odom_quat[2]
        odom.pose.pose.orientation.w = odom_quat[3]

        odom.child_frame_id = "base_footprint"
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth
        self.odom_publisher.publish(odom)
        print(odom.pose.pose)

if __name__ == '__main__':
    try:
        odom_publisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
