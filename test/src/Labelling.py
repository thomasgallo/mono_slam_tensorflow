#!/usr/bin/env python

import rospy
import sys
import math
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Point,Quaternion
from sensor_msgs.msg import PointCloud
from test.msg import DetectionArray # Array of msg Detection which gives information about an object detected in the image
from test.msg import LabelledMap    # CloudPoint and their corresponding labels
from test.msg import SlamMap        # CloudPoint and information about the current frame of the camera

## Code that simulate the behaviour of the Labelling node from the UML ##
## The node subscribe to two topics : one for the cloud map and one for the detected objects ##
## Its output is a labelled map, as a concatenation of the inputs ##

class Labelling:

    def __init__(self):

        ## Publisher and Subscribers ##
        self.sub_Map_ = rospy.Subscriber("/SLAM_Map", SlamMap, self.cb_Map)  # Subscribe to the cloudpoint (coming from the SLAM module)
        self.sub_Dtection_ = rospy.Subscriber("/Detections", DetectionArray , self.cb_Detection)    # Subscribe to the Array of detected object (coming from the tensorflow module)
        self.pub_labelledMap_ = rospy.Publisher('/LabelledMap',LabelledMap, queue_size=10)  # Publish the labelledMap

        ## Initialization of the node ##
        self.nh_ = rospy.init_node('Labelling', anonymous=True)

        ## Attributes ##
        self.map_ = PointCloud()    # Subscribed attribute
        self.detections_ = []   # Subscribed attribute
        self.labels_ = []
        self.labelledMap_ = LabelledMap()   # Published attribute
        self.cameraPosition_ = Point()
        self.cameraOrientation_ = Quaternion()

    ## Callbacks ##

    # Is called when a CloudPoint msg is received
    # Store the received value in the corresponding attribute
    def cb_Map(self,data):
        self.map_ = data.map
        self.cameraPosition_ = data.pose.pose.position
        self.cameraOrientation_ = data.pose.pose.orientation
        #rospy.loginfo(data)    #Test if the correct value has been tested

    # Is called when a DetectionArray msg is received
    # Store the received value in the corresponding attribute
    def cb_Detection(self,data):
        self.detections_ = data.detections
        #rospy.loginfo(data.detections[0].label)      #Test if the correct value has been tested

    # Transform every objects's coordinate in the global referentiel
    # For each point of the map, check if it is inside a bounding box
    # If yes, assign it the corresponding label
    def correspondingLabels(self):
        m = len(self.detections_)
        n = len(self.map_.points)
        self.labels_ = [' ' for x in range(n)]  # Initialise a null list for every point in the map
        pixels=np.matrix([320,240]) # Size of the screen
        # Position of Camera global
        T = np.matrix([self.cameraPosition_.x,self.cameraPosition_.y,self.cameraPosition_.z])
        T = np.transpose(T)
        # Rotation Matrix of Camera
        R = np.matrix([self.cameraOrientation_.x,self.cameraOrientation_.y,self.cameraOrientation_.z])
        c=np.matrix([pixels.item(0)/2,pixels.item(1)/2])	# Image center in pixels
        c=c.T
        f=35.0  # focal length
        for i in range(m): # For each object detected
            bbt = self.detections_[i].pose.pose.position.y + self.detections_[i].bounding_box_lwh.z/2  # bounding box top
            bbb = self.detections_[i].pose.pose.position.y - self.detections_[i].bounding_box_lwh.z/2  # bounding box bottom
            bbr = self.detections_[i].pose.pose.position.x + self.detections_[i].bounding_box_lwh.x/2  # bounding box right
            bbl = self.detections_[i].pose.pose.position.x - self.detections_[i].bounding_box_lwh.x/2  # bounding box left
            for j in range(n): # For each point of the map
                X = self.map_.points[j]
                # transform Global to Camera Coordinates
                XT = (R.T)*(X-T)
            	# transform Camera Coordinates to Pixel coordinates
            	Xu = np.matrix([[f*XT.item(0)/XT.item(2)],[f*XT.item(1)/XT.item(2)]])+c
                # Check if in Bounding Box
            	if Xu.item(0)>bbl and Xu.item(0)<bbr and Xu.item(1)>bbb and Xu.item(1)<bbt:
                    # Link the corresponding label to the point in the bounding box
                    self.labels_[j] = self.detections_[i].label


    ## Main function (executed in the while loop) ##
    def labelling(self):
        # If a map point is inside a boudning box --> label it
        self.correspondingLabels()
        # Assign the corresponding values to each component of the msg LabelledMap
        self.labelledMap_.map = self.map_
        self.labelledMap_.labels = self.labels_
        # Publishing in the corresponding topic
        self.pub_labelledMap_.publish(self.labelledMap_)
        self.labels_ = []   # Erase the label lists

    #Executed while the node is running
    def loop(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
                    self.labelling()
                    rate.sleep()
        rospy.spin()

if __name__ == '__main__':
    lab = Labelling()   # Initialize the state machine
    try:
        lab.loop()    # Spin the decision making code
    except:
        rospy.logerr("Error while executing labelling, please try again")
