#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Range
from sys import argv
from assisted_cleaning_solution.msg import Sensor

class range_publisher():
    def __init__(self, topic_name):
        rospy.init_node(topic_name)
        self.field_of_view, self.min_range, self.max_range = self.type_sensor(topic_name)
        self.range_sensor = 0

        self.range_publisher = rospy.Publisher(topic_name, Range, queue_size=10)
        self.sensor_subscriber = rospy.Subscriber('sensor', Sensor, self.sensor_CB, topic_name)

    def sensor_CB(self, data, args):
        if args == 'ir_back':
            self.range_sensor = data.ir_back
        elif args == 'ir_front':
            self.range_sensor = data.ir_front
        elif args == 'sonar1':
            self.range_sensor = data.sonar1
        elif args == 'sonar2':
            self.range_sensor = data.sonar2

        self.publish(args, self.field_of_view, self.min_range, self.max_range, self.range_sensor)

    def type_sensor(self, topic_name):
        if 'ir' in topic_name:
            return 0.3, 0.02, 0.15
        else:
            return 0.1, 0.02, 3

    def publish(self, topic_name, field_of_view, min_range, max_range, range_sensor):
        range_msg = Range()

        range_msg.header.frame_id = topic_name
        range_msg.radiation_type = 1
        range_msg.field_of_view = field_of_view
        range_msg.min_range = min_range
        range_msg.max_range = max_range
        range_msg.range = range_sensor

        self.range_publisher.publish(range_msg)


if __name__ == '__main__':
    try:
        print(', Value = ',argv[1], ', Length = ', len(argv), ", Type = ", type(argv[1]))
        range_publisher(argv[1])
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass