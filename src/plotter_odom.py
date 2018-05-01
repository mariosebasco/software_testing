#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix,NavSatStatus,TimeReference
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt


def callback(data):
    northing = data.pose.pose.position.x
    easting = data.pose.pose.position.y
    #rospy.loginfo("northing: [%f], easting: [%f]", northing, easting)
    plt.scatter(x=[easting], y=[northing], s = 45, c='b') 
    plt.show()

def initialization():

    rospy.init_node('plotOdomDataOnMap')

    rospy.Subscriber("odom", Odometry, callback)

    #back_alley
    # TRX = 4655469.04        #top right longitude
    # TRY = 442379.84            #top right latitude
    # BLX = 4655421.30         #bottom left longitude
    # BLY = 442255.64           #bottom left latitude

    #parking lot
    TRX = 4655437.08        #top right longitude
    TRY = 442323.22            #top right latitude
    BLX = 4655388.90         #bottom left longitude
    BLY = 442199.02           #bottom left latitude
    
    mapFile = '/home/robot/catkin_ws/src/testing/images/parking_lot.png'
    
    imgMap = 0
    #now plot the data on a graph
    plt.xlabel('easting')
    plt.ylabel('northing')
    plt.title('POSITION')

    #display the image under the graph
    #read a png file to map on
    imgMap = plt.imread(mapFile)
    implot = plt.imshow(imgMap,extent=[BLY, TRY, BLX, TRX])
    #plt.show()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    initialization()
