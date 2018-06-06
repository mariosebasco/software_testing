#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix,NavSatStatus,TimeReference
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt


def callback(data):
    northing = data.pose.pose.position.x
    easting = data.pose.pose.position.y
    rospy.loginfo("northing: [%f], easting: [%f]", northing, easting)
    plt.scatter(x=[easting], y=[northing], s = 45, c='r')
    plt.show()

def initialization():

    rospy.init_node('plotPathData')

    rospy.Subscriber("odom", Odometry, callback)

    #back_alley
    # TRX = 4655469.04        #top right longitude
    # TRY = 442379.84            #top right latitude
    # BLX = 4655421.30         #bottom left longitude
    # BLY = 442255.64           #bottom left latitude

    #parking lot
    TRX = 4655437.08        #top right 
    TRY = 442323.22            #top right 
    BLX = 4655388.90         #bottom left 
    BLY = 442199.02           #bottom left 
    
    #mapFile = '/home/robot/catkin_ws/src/testing/images/back_alley.PNG'
    mapFile = '/home/robot/catkin_ws/src/testing/images/parking_lot.png'
    
    imgMap = 0
    #now plot the data on a graph
    plt.xlabel('easting')
    plt.ylabel('northing')
    plt.title('POSITION')

    #read map file
    imgMap = plt.imread(mapFile)
    implot = plt.imshow(imgMap,extent=[BLY, TRY, BLX, TRX])

    #read path
    path_file = '/home/robot/catkin_ws/src/testing/gps_files/path.txt'
    file = open(path_file, 'r')

    #plot points on the map
    count = 1
    for line in file:
        if count % 2 == 1:
            northing = float(line)
        else:
            easting = float(line)
            plt.scatter(x=[easting], y=[northing], s = 20, c='b')
        count += 1
            
    plt.show()

    #obtain and plot odom data
    rospy.spin()

if __name__ == '__main__':
    initialization()
