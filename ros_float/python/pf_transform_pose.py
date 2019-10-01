#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pyproj as proj
import rospy
import tf
import datetime
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy

class TransformPose:
    def __init__(self):
        self.pub = rospy.Publisher('/usbl/pose_projected', Odometry, queue_size=10)
        self.pressed = False
        rospy.Subscriber("/usbl/latitude_longitude", NavSatFix, self.callback)

    def paramsReady(self):
        return rospy.has_param('/utm_zone') and\
               rospy.has_param('/origin_lon') and\
               rospy.has_param('/origin_lat') and\
               rospy.has_param('/origin_z')

    def callback(self,data):
        rospy.loginfo_once("received data lat lon usbl")
        if(self.paramsReady()):
            rospy.loginfo_once("Valid survey params found:  transform_pose starting")
            crs_wgs = proj.Proj(init='epsg:4326') # assuming you're using WGS84 geographic
            utm_zone = rospy.get_param("/utm_zone")
            crs_bng = proj.Proj(init=utm_zone) # use a locally appropriate projected CRS

            originLon = rospy.get_param("/origin_lon")
            originLat = rospy.get_param("/origin_lat")
            # then cast your geographic coordinate pair to the projected system
            xoff, yoff = proj.transform(crs_wgs, crs_bng, originLon, originLat)
            zoff = rospy.get_param("/origin_z",0)
            # input from NavSatFix
            lon = data.longitude;
            lat = data.latitude;
            x, y = proj.transform(crs_wgs, crs_bng, lon, lat)
            x = x-xoff
            y = y-yoff
            z = 0;

            # output message
            transformed_msg = Odometry()
            transformed_msg.header.stamp = data.header.stamp;
            transformed_msg.child_frame_id = "base_link"

            if x== float('Inf') or y==float('Inf'):
                pass
            else:
                transformed_msg.pose.pose.position.x=x
                transformed_msg.pose.pose.position.y=y
                transformed_msg.pose.pose.position.z=z
                transformed_msg.header.frame_id="map"
#                transformed_msg.header.frame_id="odom"

                transformed_msg.pose.covariance = [5., 0., 0., 0., 0., 0.,
                                                   0., 5., 0., 0., 0., 0.,
                                                   0., 0., 5., 0., 0., 0.,
                                                   0., 0., 0., 0., 0., 0.,
                                                   0., 0., 0., 0., 0., 0.,
                                                   0., 0., 0., 0., 0., 0.]
                self.pub.publish(transformed_msg)
                if self.pressed:
                    self.button_pub.publish(transformed_msg)
                    self.pressed = False
        else:
            rospy.loginfo_once("Receiving odom message but survey params not ready.  waiting...")

    def joyCallback(self,joyMsg):
        if(joyMsg.buttons[0]==1):
            self.pressed = True
            print("saving current location")
        return

def main():

    rospy.init_node('nav_projection_node')
    transformer = TransformPose()
    rospy.loginfo("starting")
    rospy.spin()

main()

