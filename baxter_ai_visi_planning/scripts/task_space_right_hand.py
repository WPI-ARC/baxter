#!/usr/bin/env python

import rospy
import math
import tf
from tf.transformations import *
from visualization_msgs.msg import *

class TaskSpacePublisher:
    def __init__(self, root_frame, rate):
        self.root_frame = root_frame
        self.rate = rate
        self.marker_pub = rospy.Publisher("task_space_right", MarkerArray)
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.display_task_space()
            rate.sleep()

    def display_task_space(self):
        marker_msg = MarkerArray()
        # Make bounding task space sphere
        taskspace = Marker()
        taskspace.type = Marker.SPHERE
        taskspace.ns = "task_space"
        taskspace.id = 1
        taskspace.action = Marker.ADD
        taskspace.lifetime = rospy.Duration(0.2)
        taskspace.header.stamp = rospy.Time.now()
        taskspace.header.frame_id = self.root_frame
        taskspace.scale.x = 1.0571 * 2
        taskspace.scale.y = 1.0571 * 2
        taskspace.scale.z = 1.0571 * 2
        taskspace.color.a = 0.2
        taskspace.color.r = 0.66
        taskspace.color.b = 0.66
        taskspace.color.g = 0.66
        taskspace.pose.position.x = 0.0
        taskspace.pose.position.y = 0.0
        taskspace.pose.position.z = 0.0
        taskspace.pose.orientation.x = 0.0
        taskspace.pose.orientation.y = 0.0
        taskspace.pose.orientation.z = 0.0
        taskspace.pose.orientation.w = 1.0
        marker_msg.markers.append(taskspace)
        self.marker_pub.publish(marker_msg)

def main():
    rospy.init_node("task_space_right_hand_publisher")
    rospy.loginfo("Starting the task space marker broadcaster...")
    #Get the parameters from the server
    root_frame = rospy.get_param("~root_frame", "right_lower_shoulder")
    rate = rospy.get_param("~rate", 10.0)
    TaskSpacePublisher(root_frame, rate)

if __name__ == "__main__":
    main()
