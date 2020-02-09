#!/usr/bin/env python

import rospy
import sys
import math
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud
from test.msg import DetectionArray
from test.msg import LabelledMap

## Code that simulate the behaviour of the Labelling node from the UML ##
## The node subscribe to two topics : one for the cloud map and one for the detected objects ##
## Its output is a labelled map, as a concatenation of the inputs ##

class Labelling:

    def __init__(self):

        ## Publisher and Subscribers ##
        self.sub_Map_ = rospy.Subscriber("/SLAM_Map", PointCloud, self.cb_Map)  # Subscribe to the cloudpoint (coming from the SLAM module)
        self.sub_Dtection_ = rospy.Subscriber("/Detections", DetectionArray , self.cb_Detection)    # Subscribe to the Array of detected object (coming from the tensorflow module)
        self.pub_labelledMap_ = rospy.Publisher('/LabelledMap',LabelledMap, queue_size=10)  # Publish the labelledMap

        ## Initialization of the node ##
        self.nh_ = rospy.init_node('Labelling', anonymous=True)

        ## Attributes ##
        self.map_ = PointCloud()    # Subscribed attribute
        self.detections_ = []   # Subscribed attribute
        self.labels_ = []
        self.labelledMap_ = LabelledMap()   # Published attribute

    ## Callbacks ##

    # Is called when a CloudPoint msg is received
    # Store the received value in the corresponding attribute
    def cb_Map(self,data):
        self.map_ = data
        #rospy.loginfo(data)    #Test if the correct value has been tested

    # Is called when a DetectionArray msg is received
    # Store the received value in the corresponding attribute
    def cb_Detection(self,data):
        self.detections_ = data.detections
        #rospy.loginfo(data.detections[0].label)      #Test if the correct value has been tested


    ## Main function (executed in the while loop) ##
    def labelling(self):
        # Get the label from every Detection instances in the array
        for i in range(len(self.detections_)) :
            self.labels_.append(self.detections_[i].label)
        # Assign the corresponding values to each component of the msg LabelledMap
        self.labelledMap_.map = self.map_
        self.labelledMap_.labels = self.labels_
        # Publishing in the corresponding topic
        self.pub_labelledMap_.publish(self.labelledMap_)
        #self.labels_.clear()

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
