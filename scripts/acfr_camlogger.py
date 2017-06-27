#!/usr/bin/env python
#"""@@ This node labels and saves the images of the node it subscribes too @@"""

# ROS imports
import rospy
import roslib
import rosbag
#roslib.load_manifest('acfr_camlogger')

# Msgs imports
from sensor_msgs.msg import Image as RosImage
#import tf


# Python imports
import time
import datetime
import os, sys
import numpy as np
#from math import pi


# To convert the ROS image
from cv_bridge import CvBridge, CvBridgeError
# To save the image
import cv2


class ImageLabeller:
    # """ Label and store raw images captured by the stereo camera to specific files
    #
    # """

    def __init__(self, name):

        # Name of the node
        self.name = name

        # The file converting object
        self.bridge = CvBridge()

        # Collect path to store image files to from the config variable
        try:
            #self.store_path = rospy.get_param("/image_labeller/storage_path")
            self.store_path = "~/camtest_acfr_logger"
        except(KeyError):
            # If there is an error in how the file is stored then default the file path
            self.store_path = "~/camera_images"

        # Generate the folder name based on the time variables and append to the path
        folder_label = "/%s%s" % (time.strftime("%Y%d%m_%H%M_"),
                                              int(round(time.clock() * 1000)),)
        self.store_path =  self.store_path + folder_label

        #### Message labels
        ###self.enu_msg = '/navsat/enu_datum'
        ###self.multibeam_topic = '/multibeam_scan'

        # Create the buffer for image_labels
        self.buffer = []
        # A bundle to hold multiple transforms
        self.transformBundle = None

        # Create image handles
        self.im1 = None
        self.im2 = None

        # # Create transformation frame handles
        # self.trans = None
        # self.rot = None
        # self.position = None
        #
        # # Camera transformation arrays
        # self.camera_to_structured = None
        # self.euler_camera_to_structured = None

        # The total number of matching pairs
        self.pairTotal = 0

        # # Create the origin
        # self.origin = Origin()
        # # Create the depth sensor
        # self.multi = Multibeam()
        # # Create the camera transform listener
        # #self.camera_listener = CameraListener()

        # Diagnostic info
        self.lastTime = 0
        self.currentTime = 0
        self.deltaTime = 0
        self.avgDelta = 0
        self.totalDelta = 0
        self.calledTime = 0
        self.time_string = []
        self.timestamp = None

        # Set the pair matching interval in milliseconds
        self.pairInterval = 50

        # Make the path and parent directories if they don't exist yet
        try:
            os.makedirs(os.path.expanduser(self.store_path))


        except OSError, e:
            if e.errno != 17:
                raise

        # Make the file to write to
        if os.chdir(os.path.expanduser(self.store_path)):
            pass

        # Define subscription to topic with the flag to label each camera
        rospy.Subscriber("/camera/image_raw", RosImage, self.image_callback_right)
        #rospy.Subscriber("/camera_left/image_raw", RosImage, self.image_callback_left)

        # rospy.Subscriber("/navsat/enu_datum", NavSatFix, self.origin.getOrigin)
        # rospy.Subscriber("/multibeam_scan", LaserScan, self.multi.get_depth)
##TODO is this necessary?
#        self.listener = tf.TransformListener()


    def image_callback_right(self, msg):
        # ''' This function collects and saves the raw images for left and right cameras as greyscale images. Note
        # that the color information is retained in the GBRG8 format
        #
        # :param msg: Takes the raw image as the argument
        # :return:
        # '''

        # Create the image label for the current file
        name = "RC"
        rospy.logdebug("Image callback by %s camera",name)

        # Handle time for labelling
        rospy.logdebug("Setting timestamp to the image message stamp")
        self.timestamp = msg.header.stamp #- #rospy.Time.from_sec(2)

        self.time_string = str(datetime.datetime.fromtimestamp(self.timestamp.to_sec()).time()).translate(None,':')
        print ("self.time_string")

        # Label the image
        image_label = "%s_%s_%s"% (time.strftime("PR_%Y%d%m"), self.time_string[0:6], self.time_string[7:10])+"_"+name+"16.png"

        # Try to save the raw image
        rospy.logdebug("Trying to convert image message to cv2 object")
        try:
            # Convert the raw image to a version python can work with
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "passthrough").copy()
            rospy.logdebug("Successfully converted")

        except CvBridgeError as e:
            rospy.logerr("Error converting Error, %s", e)
            print e
        else:
            rospy.logdebug("Writing the cv2 image to the folder path: %s",os.path.join(os.path.expanduser(self.store_path)))
            # Save the file with the user path if no exception
            cv2.imwrite(os.path.join(os.path.expanduser(self.store_path), image_label), cv2_img)

        # Add the image label to the buffer to compare throughout the bufeer
        rospy.logdebug("Adding the current image to the buffer to check for pairing")
        self.buffer.append(image_label)
        # Call finding pairs
        rospy.logdebug("Calling find pairs")
        self.find_pair()

    def image_callback_left(self, msg):
        # ''' This function collects and saves the raw images for left and right cameras as greyscale images. Note
        # that the color information is retained in the GBRG8 format
        #
        # :param msg: Takes the raw image as the argument
        # :return:
        # '''

        # Create the image label for the current file

        name = "LC"
        rospy.logdebug("Image callback by %s camera",name)

        # Handle time for labelling
        rospy.logdebug("Setting timestamp to the image message stamp")
        self.timestamp = msg.header.stamp #- #rospy.Time.from_sec(2)

        self.time_string = str(datetime.datetime.fromtimestamp(self.timestamp.to_sec()).time()).translate(None,':')

        # Label the image
        image_label = "%s_%s_%s"% (time.strftime("PR_%Y%d%m"), self.time_string[0:6], self.time_string[7:10])+"_"+name+"16.png"

        # Try to save the raw image
        rospy.logdebug("Trying to convert image message to cv2 object")
        try:
            # Convert the raw image to a version python can work with
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bayer_gbrg8").copy()
            rospy.logdebug("Successfully converted")

        except CvBridgeError as e:
            rospy.logerr("Error converting Error, %s", e)
            print e
        else:
            rospy.logdebug("Writing the cv2 image to the folder path: %s",os.path.join(os.path.expanduser(self.store_path)))
            # Save the file with the user path if no exception
            cv2.imwrite(os.path.join(os.path.expanduser(self.store_path), image_label), cv2_img)

        # Add the image label to the buffer to compare throughout the bufeer
        rospy.logdebug("Adding the current image to the buffer to check for pairing")
        self.buffer.append(image_label)
        ## Call finding pairs
        rospy.logdebug("Calling find pairs")
        self.find_pair()


    def find_pair(self):
        # ''' Checks a buffer of image_labels for timestamps within a range to find synced left and right camera images.
        # If found it saves and stores both labels of the pair and their  current transformation frames into a text file.
        #
        # :return:
        # '''

        # This is where the buffer is cycled through
        rospy.logdebug("Comparing images inside of the buffer")
        for i in range(0, len(self.buffer)-1):
            # Check the current image with the next image
            self.im1 = self.buffer[i]
            self.im2 = self.buffer[i+1]

            # Check for a match with a specified millisecond range
            if( abs((int(self.im1[19:22])-int(self.im2[19:22]))) <= self.pairInterval ):

                # Clear the buffer
                #rospy.logerror("Pair found so clearing buffer")
                self.buffer = []

                # If left image is first in the buffer, swap it to be second
                if(self.im1[23:25]=="LC"):
                    imNun = self.im2
                    self.im2 = self.im1
                    self.im1 = imNun
                    #print self.im1 + " and " + self.im2 + " match!"

                # rospy.logdebug("Calling the transformation broadcaster with the Multibeam timestamp")
                # self.handle_pose(self.multi.timestamp)
                # # Poll the transformation information for this pair of images
                #
                # rospy.logdebug("Trying to find transformation information from odom and basefoot_print and saving it to bundle")
                # bundle=self.find_transformation("odom","base_footprint",self.timestamp)
                # try:
                #     rospy.logdebug("Trying to save trasformation information from bundle")
                #     self.trans = bundle[0]
                #     self.rot = bundle[1]
                # except(IndexError)as e:
                #     rospy.logerr("Received index error while trying to store transformation information Error: %s",e)
                #
                # rospy.logdebug("Trying to find transformation information from odom and basefoot_print and saving it to camera_to_structured")
                # self.camera_to_structured = self.find_transformation("odom","base_footprint",self.timestamp)

                # print self.camera_to_structured


                # # Begin writing the image labels and transformation information
                # if (self.all_started() == True):
                #     rospy.logdebug("Starting variables initalized")
                #     rospy.logdebug("Images %s and %s match")
                #     #print self.im1 +" and " + self.im2 +" match!"
                #
                #     self.write_format()
                #     rospy.loginfo("Current pair total: %s",self.pairTotal)
                #     self.pairTotal += 1


                rospy.logdebug("#################### End of image pairing cycle #####################")
                break
            else:
                #print self.im1 +" and " + self.im2 +" don't match!"
                rospy.logdebug('###################### Images did not match ##############################')

# ########################################
#     def find_transformation(self,parent_id,child_id,timestamp):
#         #print "Find Transformation ( "+parent_id+", "+child_id+")"
#         rospy.logdebug("Trying to find transformation from %s to %s",parent_id,child_id)
#         # now = rospy.Time.now()
#         now = timestamp
#         # Request latest available transformation frame
#         #now = rospy.Time.now()
#         rospy.logdebug("Setting time, to check for transformation between %s and %s, to latest common time")
#         try:
#
#             now = self.listener.getLatestCommonTime(parent_id,child_id)
#             rospy.logdebug("Latest common time between %s and %s is %s",parent_id,child_id,now)
#         except(Exception) as e:
#             rospy.logerr("Could not get latest common time between %s as %s since tf tree link not yet established between them",parent_id,child_id)
#
#         # Try to find the transformation between parent and child
#         try:
#             # Get origin to AUV transformation waiting 1ms for the latest available frame
#             self.listener.waitForTransform(parent_id, child_id, now, rospy.Duration(nsecs=100.0))
#
#             # Collect transformation information
#             (trans, rot) = self.listener.lookupTransform(parent_id, child_id, now)
#
#             rospy.logdebug("Found transformation successfully Trans: %s Rot: %s",trans,rot)
#
#             return [trans, rot]
#
#         except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
#             rospy.logerr("Could not find transformation from %s to %s due to Error: %s (So returning none)",parent_id,child_id,e)
#             return [None]
    #
    # def write_format(self):
    #     ''' Writes the current image_labels and transformation information to a .txt file
    #
    #     :return:
    #     '''
    #
    #     # Update the position in Lat Lon
    #     self.position = llUTM.UTMtoLL(23, self.origin.northing + self.trans[1], self.origin.easting + self.trans[0],
    #                                   self.origin.zone)
    #     rospy.logdebug("New position in LL updated")
    #     # Try to write the image_labels and the transformation info to the file
    #     try:
    #         rospy.logdebug("Writing to stereo_pose_est.data file now")
    #
    #         # Write things to file once here
    #         if(self.pairTotal == 0):
    #             rospy.loginfo("Writing initial lines of stereo_pose_est.data file now")
    #             self.f.write("% STEREO_POSE_FILE VERSION 2"+"\n")
    #             self.f.write("ORIGIN_LATITUDE "+str(self.origin.latZero)+"\n")
    #             self.f.write("ORIGIN_LONGITUDE "+str(self.origin.lonZero)+"\n")
    #
    #         try:
    #             euler_camera_to_structured = tf.transformations.euler_from_quaternion(self.camera_to_structured[1])
    #         except(IndexError)as e:
    #             rospy.logerr("Euler_camera_to_structured can't be calculated due to Error: %s",e)
    #         self.f.write(str(self.pairTotal)+" "+str(self.timestamp)+" ")
    #         #self.f.write(
    #         #    " ".join("%s" % x for x in range(len(self.trans))) + " ".join("%s" % x for x in range(len(self.rot))))
    #         self.f.write(str(self.position[0])+" "+str(self.position[1])+" ")
    #         self.f.write(str(self.camera_to_structured[0]).translate(None, '[(,)]') + " ")
    #         self.f.write(str(euler_camera_to_structured).translate(None, '(,)')+" ")
    #
    #         self.f.write(self.im1 + " " + self.im2+" ")
    #
    #         self.f.write(str(self.multi.lastdepth) + ' ')
    #
    #         self.f.write('0 ')
    #         self.f.write('0 '+"\n")
    #
    #     except UnboundLocalError as e:
    #         rospy.logerr("Tried to write but could not because of Error: %s",e)
    def toSeconds(self,time_str):
        combinedTime = ""
        ((int(self.time_string[0:1]) * 10 + int(self.time_string[1:2])) * 60
         + int(self.time_string[2:4])) * 60 + int(self.time_string[4:10])
        # Get a string of hours to minutes plus the current minutes
        combinedTime = str(60*(int(time_str[0:1])*10+int(time_str[1:2])))+int(time_str[2:4])
        # Get a string of minutes to seconds adding current seconds
        combinedTime = combinedTime


    # def all_started(self):
    #     ''' Check that all initially None variables have values and are ready to start
    #
    #     :return: bool
    #     '''
    #     rospy.logdebug("Checking that startup variables are set")
    #     if(self.im1 != None and self.im2 != None and self.trans != None
    #        and self.origin.lat != 0  and self.camera_to_structured != None):
    #         return True
    #     else:
    #         rospy.logerr("Can't write to file, not all startup variables initialized")
    #         rospy.logerr("1 Image1: %s",self.im1)
    #         rospy.logerr("2 Image2: %s",self.im2)
    #         rospy.logerr("3 Odom to Base Translation: %s",self.trans)
    #         rospy.logerr("4 Origin Latitude: %s",self.origin.lat)
    #         rospy.logerr("5 (Camera_to_structured: %s",self.camera_to_structured)
    #         rospy.logerr("6 (Depth): %s",self.multi.lastdepth)
    #         return False

    def calculateTime(self, currTime, show):
        # ''' Calculates time variables to keep track of elapsed time for operations based on the current number of pairs
        #
        # :param currTime: Integer value
        # :return:
        # '''
        if(self.calledTime == 0):
            self.lastTime = currTime
            self.calledTime += 1


        self.calledTime += 1
        self.currentTime = currTime
        self.deltaTime = self.currentTime-self.lastTime
        self.totalDelta += self.deltaTime
        self.lastTime = currTime
        try:
            self.avgDelta = self.totalDelta/self.calledTime
        except(ZeroDivisionError):
            #self.avgTime = 0
            #No pairs yet

            pass
        if(show == True):
            print "Delta T: " + str(self.deltaTime) + "ms"
            print "Average Delta :" + str(self.avgDelta) + "ms"
            print "Current T : " + str(self.currentTime) + "ms"
            print "Last T: " + str(self.lastTime) + "ms"
    def getTimestamp(self):
        return self.timestamp
    # def handle_pose(self,timestamp):
    #     ''' Broadcast the wanted transformations
    #
    #     :param timestamp: the current time
    #     :return:
    #     '''
    #
    #     # Create the broadcasting object
    #     br = tf.TransformBroadcaster()
    #
    #     # Set the time to the current ros clock time
    #     now = rospy.Time.now()
    #     #now = timestamp
    #
    #     # Try to send the transformations
    #     try:
    #
    #         quat = tf.transformations.quaternion_from_euler(-pi / 2, pi, 0, 'szxy')
    #
    #         br.sendTransform((0, 0, 0),
    #                          quat, now, "odom", "structured")
    #
    #         # Insert transform from frame base_footprint to camera
    #     except(TypeError) as e:
    #         rospy.logerr("Transformation odom to structured not broadcasted due to Error: %s",e)
    #
    #     try:
    #         rot = tf.transformations.quaternion_from_euler(-pi / 2, pi, 0, 'szxy')
    #
    #         br.sendTransform((0, 0, 0),
    #                          rot, now, "camera", "base_footprint")
    #     except(TypeError) as e:
    #         rospy.logerr("Transformation camera to base_footprint not broadcasted due to Error: %s",e)


# class Origin:
#
#     def __init__(self):
#         self.gps_datum_topic = "/navsat/enu_datum"
#
#         self.lat = 0
#         self.lon = 0
#         self.zone = 0
#         self.easting = 0
#         self.northing = 0
#         self.latZero = None
#         self.lonZero = None
#
#     def getOrigin(self,msg):
#         # Check for the initial latitude and longitude to write to file
#         rospy.loginfo("Getting origin and setting variables")
#         if(self.latZero == None and self.lonZero == None):
#             self.latZero = msg.latitude
#             self.lonZero = msg.longitude
#
#
#         self.lat = msg.latitude
#         self.lon = msg.longitude
#         (self.zone, self.easting, self.northing) = llUTM.LLtoUTM(23, self.lat, self.lon)
#         #print("Got Bag Origin. Easting: ", self.easting, " Northing: ", self.northing, " Zone: ", self.zone)
#         #print("Latitude: ", self.lat, ' Longitude: ', self.lon)
#         rospy.loginfo("Got Bag Origin. Easting: %s Northing: %s Zone: %s", self.easting, self.northing, self.zone)
#         rospy.loginfo("Latitude: %s Longitude: %s", self.lat, self.lon)
#         return
#
#
# class Multibeam:
#
#     def __init__(self):
#         self.lastdepth = None
#         self.updated = False
#         self.timestamp = None
#
#     def get_depth(self, msg):
#         rospy.logdebug("Getting depth and setting variables")
#         self.timestamp = msg.header.stamp
#         cos_angles = np.cos(np.arange(msg.angle_min, msg.angle_max, msg.angle_increment))
#         z_values = np.multiply(cos_angles, np.array(msg.ranges))
#         self.lastdepth = np.mean(z_values[np.nonzero(z_values)])
#         self.updated = True
#         rospy.logdebug("Depth variables set")


if __name__ == '__main__':
    try:
        rospy.init_node('acfr_camlogger')
        labeller = ImageLabeller(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
