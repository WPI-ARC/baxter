#!/usr/bin/env python

import rospy
import math
import tf
from tf.transformations import *
from visualization_msgs.msg import *

class TableObstaclePublisher:
    def __init__(self, root_frame, rate):
        self.root_frame = root_frame
        self.rate = rate
        self.marker_pub = rospy.Publisher("task_space_right", MarkerArray)
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.display_table_and_obstacle()
            rate.sleep()

    def display_table_and_obstacle(self):
        marker_msg = MarkerArray()
        # Make obstacle
        obstacle = Marker()
        obstacle.type = Marker.CUBE
        obstacle.ns = "obstacle"
        obstacle.id = 2
        obstacle.action = Marker.ADD
        obstacle.lifetime = rospy.Duration(0.2)
        obstacle.header.stamp = rospy.Time.now()
        obstacle.header.frame_id = self.root_frame
        obstacle.scale.x = 0.27
        obstacle.scale.y = 0.34
        obstacle.scale.z = 0.34
        obstacle.color.a = 1.0
        obstacle.color.r = 1.0
        obstacle.color.g = 0.0
        obstacle.color.b = 0.0
        obstacle.pose.position.x = 0.65
        obstacle.pose.position.y = -0.2
        obstacle.pose.position.z = 0.05
        obstacle.pose.orientation.x = 0.0
        obstacle.pose.orientation.y = 0.0
        obstacle.pose.orientation.z = 0.0
        obstacle.pose.orientation.w = 1.0
        marker_msg.markers.append(obstacle)
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
	# Make Baxter-Box
        baxterBox = Marker()
        baxterBox.type = Marker.CUBE
        baxterBox.ns = "baxter_box"
        baxterBox.id = 2
        baxterBox.action = Marker.ADD
        baxterBox.lifetime = rospy.Duration(0.2)
        baxterBox.header.stamp = rospy.Time.now()
        baxterBox.header.frame_id = self.root_frame
        baxterBox.scale.x = 0.3
        baxterBox.scale.y = 0.6
        baxterBox.scale.z = 1.7
        baxterBox.color.a = 1.0
        baxterBox.color.r = 0.66
        baxterBox.color.b = 0.66
        baxterBox.color.g = 0.66
        baxterBox.pose.position.x = 0.0
        baxterBox.pose.position.y = 0.0
        baxterBox.pose.position.z = 0.0
        baxterBox.pose.orientation.x = 0.0
        baxterBox.pose.orientation.y = 0.0
        baxterBox.pose.orientation.z = 0.0
        baxterBox.pose.orientation.w = 1.0
        marker_msg.markers.append(baxterBox)
        self.marker_pub.publish(marker_msg)

def main():
    rospy.init_node("obstacle_right_hand_publisher")
    rospy.loginfo("Starting the table marker broadcaster...")
    #Get the parameters from the server
    root_frame = rospy.get_param("~root_frame", "base")
    rate = rospy.get_param("~rate", 10.0)
    TableObstaclePublisher(root_frame, rate)

if __name__ == "__main__":
    main()
