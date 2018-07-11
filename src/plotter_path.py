#!/usr/bin/env python
import rospy
import sys
from nav_msgs.msg import Odometry
import cv2
import Image, ImageDraw
import matplotlib
import matplotlib.pyplot as plt
import matplotlib
from threading import Thread

updatePlot = False
northing = 0
easting = 0

def callback(data):
    global northing
    global easting
    global updatePlot
    
    northing = data.pose.pose.position.x
    easting = data.pose.pose.position.y
    rospy.loginfo("northing: [%f], easting: [%f]", northing, easting)
    updatePlot = True
    
def Spin():
    rospy.spin()
            
if __name__ == '__main__':
    rospy.init_node('plotPathData')

    rospy.Subscriber("odom", Odometry, callback)

    # #back_alley
    # # TRX = 4655469.04        #top right longitude
    # # TRY = 442379.84            #top right latitude
    # # BLX = 4655421.30         #bottom left longitude
    # # BLY = 442255.64           #bottom left latitude
    
    # #parking lot
    TRX = 4655437.08        #top right 
    TRY = 442323.22            #top right 
    BLX = 4655388.90         #bottom left 
    BLY = 442199.02           #bottom left 
    
    img = cv2.imread('/home/robot/catkin_ws/src/testing/images/parking_lot.png')
    height, width, depth = img.shape
    
    #read path
    path_file = '/home/robot/catkin_ws/src/testing/gps_files/path.txt'
    file = open(path_file, 'r')
        
    #plot points on the map
    count = 1
    for line in file:
        if count % 2 == 1:
            path_northing = float(line)
            pixel_x = int(height*((TRX - path_northing)/(TRX - BLX)))
        else:
            path_easting = float(line)
            pixel_y = int(width - width*((TRY - path_easting)/(TRY - BLY)))
            cv2.circle(img, (pixel_y, pixel_x), 5, (0, 0, 255), -1)
            cv2.imshow('image', img)
            cv2.waitKey(1)
        count = count + 1

    #obtain and plot odom data
    Thread(target = Spin).start()

    while not rospy.is_shutdown():
        if updatePlot:
            pixel_x = int(height*((TRX - northing)/(TRX - BLX)))
            pixel_y = int(width - width*((TRY - easting)/(TRY - BLY)))
            cv2.circle(img, (pixel_y, pixel_x), 5, (255, 0, 0), -1)
            updatePlot = False
        cv2.imshow('image', img)
        cv2.waitKey(1)

