#!/usr/bin/env python3

#VISUALIZATION OF WAYPOINTS COMPUTED IN THE ROADMAP

#import libraries
import numpy as np
import pandas as pd
import math as m
import time as tm
import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

markerArray = MarkerArray()

def read_WPScsv():

    rospy.loginfo("Read the csv to get waypoints computed by the roadmap")
    df = pd.read_csv("src/navigation/script/csv_tables/WaypointsPlanner.csv", header=None)
    WPS = np.array(df)

    return WPS

def pub_wp(i,WPS):

    rospy.loginfo("Publishing waypoint [%f,%f]", WPS[i-1][0], WPS[i-1][1])

    marker = Marker()

    marker.header.frame_id = "world"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.id = i
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = WPS[i-1][0]
    marker.pose.position.y = WPS[i-1][1]
    marker.pose.position.z = 0

    markerArray.markers.append(marker)

def main():

    topic = 'visualization_Roadmap'
    WPS_pub = rospy.Publisher(topic, MarkerArray, queue_size=150)
    rospy.init_node('Roadmap_visualizer', anonymous=True)

    rospy.loginfo("Publishing intermediate waypoints")

    WPS = read_WPScsv()

    rospy.loginfo("Found %i intermediate waypoints in the file", len(WPS))

    for i in range(len(WPS)):
        
            pub_wp(i+1,WPS)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        WPS_pub.publish(markerArray)

        rate.sleep()


if __name__ == '__main__':
    main()