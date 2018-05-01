#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix,NavSatStatus,TimeReference
import matplotlib.pyplot as plt


def callback(data):
    #rospy.loginfo("lat [%f], lon [%f], status [%i]", data.latitude, data.longitude, data.status.status)
    plt.scatter(x=[data.longitude], y=[data.latitude], s = 45, c='r') #sets your home position
    plt.show()

def initialization():

    rospy.init_node('plotGpsDataOnMap')

    rospy.Subscriber("fix", NavSatFix, callback)

    #back alley
    # TRX = -87.696282      #top right longitude
    # TRY = 42.049161       #top right latitude
    # BLX = -87.697778      #bottom left longitude
    # BLY = 42.048722       #bottom left latitude

    #parking lot
    TRX = -87.696963      #top right longitude
    TRY = 42.048869       #top right latitude
    BLX = -87.698459      #bottom left longitude
    BLY = 42.048426       #bottom left latitude

    mapFile = '/home/robot/catkin_ws/src/testing/images/parking_lot.png'
    
    imgMap = 0
    #now plot the data on a graph
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.title('POSITION (in Decimal Degrees)')

    #display the image under the graph
    #read a png file to map on
    imgMap = plt.imread(mapFile)
    implot = plt.imshow(imgMap,extent=[BLX, TRX, BLY, TRY])
    #plt.show()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    initialization()
