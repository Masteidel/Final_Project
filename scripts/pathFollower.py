import rospy, tf, numpy, math, random, Queue
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import GridCells
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler

#Upon receipt of a path, instructs the robot to drive to the waypoints in the proper order
def followPath(path):
    print "Path Received"
    wayPoints = path.poses
    for wayPoint in wayPoints: #iterate through the path
        print "Going to next waypoint"
        navToPose(wayPoint) #go to the waypoint
    print "AT TARGET"

#drive to a goal, given as a poseStamped message
def navToPose(goal):
    global currX
    global currY
    global currAng

    getCurrentPos()

    quat = goal.pose.orientation
    euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

    finalAng = euler[2]#angle we need to end up at
    targetX = goal.pose.position.x
    targetY = goal.pose.position.y
    
    targetAng = math.atan2(targetY-currY, targetX-currX)#angle towards target

    print "spin!"
    rotateToAngle(targetAng)
    print "move!"
    driveStraight(1, (math.sqrt(((currX-targetX)**2) + ((currY-targetY)**2))))
    print "spin!"
    rotateToAngle(finalAng)
    print "done"

#funtion to get current position
def getCurrentPos():
    #current position and orientation:
    global currX
    global currY
    global currAng

    rate = rospy.Rate(10.0)
    try:
        (trans,quat) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
        #the transform array is fine for x and y
        currX = trans[0]
        currY = trans[1]
        #need to use the Euler-ized quaternion
        euler = tf.transformations.euler_from_quaternion(quat)
        currAng = normAngle(euler[2])#note that it gets converted right here to be between 0 and 2 pi, prevents issues later
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print "TF Exception"
        
def stop():
    #STOP:
    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    
    rospy.sleep(rospy.Duration(1, 0))

#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
    global currX
    global currY
    
    #get starting position
    getCurrentPos()

    startX = currX
    startY = currY

    while(math.sqrt(((startX-currX)**2) + ((startY-currY)**2)) < distance):
        getCurrentPos()
        #publish to Twist:
        twist = Twist()
        twist.linear.x = speed; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)


#Normalizes the angle from 0-2pi (its a little weird straight from the 'bot)
def normAngle(angle):
    #2pi or not 2pi, that is the question
    if (angle >=  0):#all good!
        return angle
    else:
        return (2*math.pi)+angle#this compensates for the fact that the
                                #angle is starting at -pi and working to 0
    
#Accepts an angle and makes the robot rotate to it.
def rotateToAngle(angle):
    global currAng
    getCurrentPos()
    angle = normAngle(angle)#make it between pi and 2pi
    
    while (currAng > angle):
        getCurrentPos()#could have prob set up a timer to do this, might have been worth it since 
                       #forgetting this call is the number one "unsolvable" bug right now
        #publish to Twist:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = -1
        pub.publish(twist)
    while (currAng < angle):
        getCurrentPos()
        #publish to Twist:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 1
        pub.publish(twist)
    stop()

# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('pathFollower')

    # These are global variables. Write "global <variable_name>" in any other function to gain access to these global variables 
    global pub
    global pose
    global odom_tf
    global odom_list
    global stopFlag
   
    stopFlag = 0
    #global position/orientation variables
    global currX
    global currY
    global currAng
    
    currX = 0
    currY = 0
    currAng = 0

    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10) # Publisher for commanding robot motion
    pathSub = rospy.Subscriber('/aStar_Path', Path, followPath, queue_size=1)
    # Use this object to get the robot's Odometry 
    odom_list = tf.TransformListener()
    
    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))
    print "Starting Path Follower"

    #make the robot keep doing something...
    # rospy.Timer(rospy.Duration(...), timerCallback)

    # Make the robot do stuff...
    
    while(1):
        pass