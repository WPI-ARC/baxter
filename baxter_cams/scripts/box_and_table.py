#!/usr/bin/env python

import rospy
import math
import tf
from tf.transformations import *
from visualization_msgs.msg import *

class TableBoxPublisher:
    def __init__(self, root_frame, rate):
        self.root_frame = root_frame
        self.rate = rate
        self.marker_pub = rospy.Publisher("table_markers", MarkerArray)
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.display_table_and_box()
            rate.sleep()

    def display_table_and_box(self):
        marker_msg = MarkerArray()
        # Make table-top
        tabletop = Marker()
        tabletop.type = Marker.CUBE
        tabletop.ns = "table"
        tabletop.id = 1
        tabletop.action = Marker.ADD
        tabletop.lifetime = rospy.Duration(0.2)
        tabletop.header.stamp = rospy.Time.now()
        tabletop.header.frame_id = self.root_frame
        tabletop.scale.x = 0.5
        tabletop.scale.y = 1.21
        tabletop.scale.z = 0.025
        tabletop.color.a = 1.0
        tabletop.color.r = 0.66
        tabletop.color.b = 0.66
        tabletop.color.g = 0.66
        tabletop.pose.position.x = 0.8
        tabletop.pose.position.y = 0.0
        tabletop.pose.position.z = -0.14
        tabletop.pose.orientation.x = 0.0
        tabletop.pose.orientation.y = 0.0
        tabletop.pose.orientation.z = 0.0
        tabletop.pose.orientation.w = 1.0
        marker_msg.markers.append(tabletop)
        # Make box
        box = Marker()
        box.type = Marker.CUBE
        box.ns = "box"
        box.id = 2
        box.action = Marker.ADD
        box.lifetime = rospy.Duration(0.2)
        box.header.stamp = rospy.Time.now()
        box.header.frame_id = self.root_frame
        box.scale.x = 0.183
        box.scale.y = 0.256 
        box.scale.z = 0.095
        box.color.a = 1.0
        box.color.r = 1.0
        box.color.g = 0.0
        box.color.b = 0.0
        box.pose.position.x = 0.8
        box.pose.position.y = 0.0
        box.pose.position.z = (0.025 / 2) + (0.095 / 2) - 0.14
        box.pose.orientation.x = 0.0
        box.pose.orientation.y = 0.0
        box.pose.orientation.z = 0.0
        box.pose.orientation.w = 1.0
        marker_msg.markers.append(box)
        self.marker_pub.publish(marker_msg)

def main():
    rospy.init_node("box_and_table_publisher")
    rospy.loginfo("Starting the table marker broadcaster...")
    #Get the parameters from the server
    root_frame = rospy.get_param("~root_frame", "base")
    rate = rospy.get_param("~rate", 10.0)
    TableBoxPublisher(root_frame, rate)

if __name__ == "__main__":
    main()
