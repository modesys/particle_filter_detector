#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pyproj as proj
import rospy
import tf
import tf2_ros
import datetime
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
import geometry_msgs.msg

class TransformGPSPose:
    def __init__(self):
        self.pub = rospy.Publisher('/gps/pose_projected', Odometry, queue_size=10)
        self.pressed = False
        rospy.Subscriber("/gps/latitude_longitude", NavSatFix, self.callback)
        self.br = tf2_ros.StaticTransformBroadcaster()

    def paramsReady(self):
        return rospy.has_param('/utm_zone') and\
               rospy.has_param('/origin_lon') and\
               rospy.has_param('/origin_lat') and\
               rospy.has_param('/origin_z')

    def callback(self,data):
        rospy.loginfo_once("received data lat lon usbl")
        if(self.paramsReady()):
            rospy.loginfo_once("Valid GPS params found")
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

            static_transformStamped = geometry_msgs.msg.TransformStamped()

#            static_transformStamped.header.stamp = rospy.Time.now()
#            static_transformStamped.header.frame_id = "utm"
#            static_transformStamped.child_frame_id = "map"

#            static_transformStamped.transform.translation.x = xoff
#            static_transformStamped.transform.translation.y = yoff
#            static_transformStamped.transform.translation.z = 0

#            quat = tf.transformations.quaternion_from_euler(0,0,0)
#            static_transformStamped.transform.rotation.x = quat[0]
#            static_transformStamped.transform.rotation.y = quat[1]
#            static_transformStamped.transform.rotation.z = quat[2]
#            static_transformStamped.transform.rotation.w = quat[3]

#            self.br.sendTransform(static_transformStamped)

#            # output message
#            transformed_msg = Odometry()
#            transformed_msg.header.stamp = data.header.stamp;
#            transformed_msg.child_frame_id = "base_link"

#            if x== float('Inf') or y==float('Inf'):
#                pass
#            else:
#                transformed_msg.pose.pose.position.x=x
#                transformed_msg.pose.pose.position.y=y
#                transformed_msg.pose.pose.position.z=z
#                transformed_msg.header.frame_id="map"
#                transformed_msg.pose.covariance = [5., 0., 0., 0., 0., 0.,
#                                                   0., 5., 0., 0., 0., 0.,
#                                                   0., 0., 5., 0., 0., 0.,
#                                                   0., 0., 0., 0., 0., 0.,
#                                                   0., 0., 0., 0., 0., 0.,
#                                                   0., 0., 0., 0., 0., 0.]
#                self.pub.publish(transformed_msg)
#                if self.pressed:
#                    self.button_pub.publish(transformed_msg)
#                    self.pressed = False
#        else:
#            rospy.loginfo_once("Receiving GPS message but survey params not ready.  waiting...")

    def joyCallback(self,joyMsg):
        if(joyMsg.buttons[0]==1):
            self.pressed = True
            print("saving current location")
        return

def main():

    rospy.init_node('gps_projection_node')
    transformer = TransformGPSPose()
    rospy.loginfo("starting")
    rospy.spin()

main()
