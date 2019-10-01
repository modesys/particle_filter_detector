#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TwistStamped, Pose, Point, Vector3, Quaternion
from std_msgs.msg import Header, ColorRGBA, String
from nav_msgs.msg import Odometry

class TrajectoryInteractiveMarkers:

    def __init__(self):
        self.count = 0
        #rospy.Subscriber("/filter/fluid_pressure/Odom_depth", TwistStamped, self.MarkerCallback)
        # Who is receiving: output
        rospy.Subscriber("/Odometry/filtered", String, self.MarkerCallback)
        rospy.sleep(0.5)

    def MarkerCallback(self, msg):
        self.waypoints = msg
        #self.a = [1, 1, 1]
        self.a = list()
        self.a.append(self.waypoints.twist.linear.x)
        self.a.append(self.waypoints.twist.linear.y)
        self.a.append(self.waypoints.twist.linear.z)
        self.showRViz()

    def showRViz(self):
        self.marker = Marker()
        # Who is sending: Input
        self.marker_publisher = rospy.Publisher('/filter/fluid_pressure/Odom_depth', Marker, queue_size=5)
        self.marker = Marker(
                    type=Marker.SPHERE,
                    id=0,
                    lifetime=rospy.Duration(1000),
                    pose=Pose(Point(self.a[0]/10**5, self.a[1]/10**5, self.a[2]/10**5), Quaternion(0, 0, 0, 1)),
                    scale=Vector3(0.05, 0.05, 0.05),
                    header=Header(frame_id='map'),
                    color=ColorRGBA(0.0, 2.0, 0.0, 0.0))
        self.count+=1
        self.marker.id = self.count
        self.marker_publisher.publish(self.marker)
        rospy.loginfo('msg published')


        while not rospy.is_shutdown():
            arr = Markers()
            arr.header.frame_id = "/map"
            arr.header.stamp = rospy.Time.now()
            msg.pose.orientation.x = 0.5
            msg.pose.orientation.y = 0.5
            msg.pose.orientation.z = 0.5
            msg.pose.orientation.w = 0.0
            msg.size = 1
            msg.color.r = 25/255.0
            msg.color.g = 255/255.0
            msg.color.b = 248/255.0
            msg.color.a = 1.0
            self.marker_publisher.publish(self.marker)


    if __name__ == '__main__':
        rospy.init_node("trajectory_interactive_marker_node", anonymus=True)
        trajectory_interactive_markers = TrajectoryInteractiveMarkers()
        rospy.sleep(0.5)
        rospy.spin()

# We could also create a starting point and a goal (end of the mission) to which send the float
# This could be good to establish a proper path_planning



