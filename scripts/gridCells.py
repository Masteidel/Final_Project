#!/usr/bin/env python
import rospy, tf, numpy, math, random
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion

def cellPublish():
    pub = rospy.Publisher('gridCell', GridCells, queue_size=10)
    rospy.init_node('cellPublish', anonymous=True)
    rate = rospy.Rate(10) # 10hz



    while not rospy.is_shutdown():
        global seqNum
        #create header:
        head = Header()
        head.seq = seqNum
        seqNum = seqNum + 1
        head.stamp = rospy.get_rostime()
        head.frame_id = "map"

        points = pointList()#get the points

        cells = GridCells()#create the message
        #fill the message with the necessary data
        cells.header = head
        cells.cell_width = 1
        cells.cell_height = 1
        cells.cells = points
        
        pub.publish(cells)
        
        rate.sleep()

def pointList():
    points = []
    for i in range(0, 10):
        points.append(randPoints())
    return points

def randPoints():
    random.seed(a=None)
    msg = Point()
    msg.x = random.random()*10 - 5
    msg.y = random.random()*10 - 5
    msg.z = 0
    return msg

if __name__ == '__main__':
    global seqNum
    seqNum = 0
    try:
        cellPublish()
    except rospy.ROSInterruptException:
        pass
