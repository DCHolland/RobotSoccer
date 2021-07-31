#!/usr/bin/env python

import roslib
import numpy
import math
roslib.load_manifest('ballfinder')
import rospy
import cv2
from sensor_msgs.msg import Image, LaserScan
from ballfinder.msg import assn3
from cv_bridge import CvBridge, CvBridgeError
class Detector:
  
    def __init__(self):
        # The image publisher is for debugging and figuring out
        # good color values to use for ball detection
        self.impub = rospy.Publisher('/ball_detector/image', Image, queue_size=1)
        self.locpub = rospy.Publisher('/ball_detector/ball_location', assn3,queue_size=1)
        self.bridge = CvBridge()
        self.bearing = -1
        self.distance = -1
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.handle_image, queue_size=1, buff_size=2**24)
        rospy.Subscriber('/scan', LaserScan, self.handle_scan)

    def handle_image(self, msg):      
	try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print e
        # Find the average column of the bright yellow pixels
        # and store as self.bearing. Store -1 if there are no
        # bright yellow pixels in the image.
        # Feel free to change the values in the image variable
        # in order to see what is going on
        # Here we publish the modified image; it can be
        # examined by running image_view
        cols = 0;
	li = []
        (rows,columns,channels) = image.shape
        for r in range (rows/2,rows,5):
            for c in range (0,columns,4):
            	if image[r,c,0] >=0 and image[r,c,0]<=110 and image[r,c,1] >=150 and image[r,c,1]<=255 and image[r,c,2] >=100 and image[r,c,2]<=255:
			li.append(int(c));	                    
        		cols=cols+1
        		image[r,c]=[0,0,255]

	if not math.isnan(numpy.median(numpy.array(li))):        
		self.bearing=int(numpy.median(numpy.array(li)))
	print("COLS  "+str(cols))
	if cols<110:
		self.bearing=-1
	for r1 in range (0, rows):
		if self.bearing!=-1:		
			image[r1,self.bearing] = [255,0,0]
	self.impub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))

	if math.isnan(self.bearing):
		self.bearing=-1
	if math.isnan(self.distance):
		self.distance=-1
	print self.bearing
	print self.distance

  
    def handle_scan(self, msg):
        # If the bearing is valid, store the corresponding range
        # in self.distance.  Decide what to do if range is NaN.
	self.distance = msg.ranges[-self.bearing]
	if math.isnan(msg.ranges[-self.bearing]):
		self.distance=-1;

              
  
    def start(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            location = assn3()
            location.bearing = self.bearing
            location.distance = self.distance
            self.locpub.publish(location)
	    print(self.bearing)
	    print(self.distance);
            rate.sleep()
  
rospy.init_node('ball_detector')
detector = Detector()
detector.start()
