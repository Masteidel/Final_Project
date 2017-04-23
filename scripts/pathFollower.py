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
        print wayPoint
        navToPose(wayPoint) #go to the waypoint
    print "AT TARGET"

#drive to a goal, given as a poseStamped message
def navToPose(goal):
    global pose
    global xPosition
    global yPosition
    global theta
    
    goalPoseX = goal.pose.position.x    #x position of the goal
    goalPoseY = goal.pose.position.y    #y position of the goal
    odomW = goal.pose.orientation
    q = [odomW.x, odomW.y, odomW.z, odomW.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    goalPoseAng = yaw                   #orientation of goal

    initialX = xPosition                #Starting x position of turtlebot
    initialY = yPosition                #Starting y position of turtlebot

    initialAng = math.radians(theta)

    #Rotate towards goal
    if((goalPoseX - initialX) == 0):
        if((goalPoseY - initialY) > 0):
            print "spin!"
            rotate((math.pi/2.0) - initialAng)
        elif((goalPoseY - initialY) < 0):
            print "spin!"
            rotate(-(math.pi/2.0) - initialAng)
    else:
        print "spin!"
        rotate(math.atan2((goalPoseY - initialY), (goalPoseX - initialX)) - initialAng)
    #Drive towards goal
    print "move!"
    driveStraight(0.2, math.sqrt(math.pow((goalPoseX - initialX), 2) + math.pow((goalPoseY - initialY), 2)))
    initialAng = math.radians(theta)    #Heading of turtlebot after reaching desired location
    #Rotate to pose
    if((goalPoseAng - initialAng) != 0):
        if((goalPoseAng - initialAng) > math.pi):
            print "spin!"
            rotate((goalPoseAng - initialAng) - 2*math.pi)
        elif((goalPoseAng - initialAng) < -math.pi):
            print "spin!"
            rotate((goalPoseAng - initialAng) + 2*math.pi)
        else:
            print "spin!"
            rotate(goalPoseAng - initialAng)
    print "done"
    pass

#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
    global pose
    global pose
    global xPosition
    global yPosition
    global theta

    #Initial x and y positions of the turtlebot
    initialX = xPosition
    initialY = yPosition

    #Create two Twist messages
    drive_msg = Twist()
    stop_msg = Twist()

    #Populate messages with data
    drive_msg.linear.x = speed
    stop_msg.linear.x = 0
    atTarget = False
    while(not atTarget and not rospy.is_shutdown()):
        #Continously find the distance travelled from starting position
        currentX = xPosition
        currentY = yPosition
        currentDistance = math.sqrt(math.pow((currentX - initialX), 2) + math.pow((currentY - initialY), 2))
        #Drive until the robot has reached its desired positon
        if(currentDistance >= distance):
            atTarget = True
            pub.publish(stop_msg)
        else:
            pub.publish(drive_msg)
            rospy.sleep(0.15)

#Accepts an angle and makes the robot rotate to it.
def rotate(angle):
    global map_list
    global pose
    global theta
    #Check if angle is within acceptable range
    if(angle > math.pi or angle < -math.pi):
        print "angle is too large or too small"
    else:
        vel = Twist()
        done = False

        #Initial heading
        initialThetaRad = math.radians(theta)

        #Determine which direction to rotate
        if(angle > 0):
            vel.angular.z = 1
        else:
            vel.angular.z = -1
        while(not done and not rospy.is_shutdown()):
            #Continuously update current heading and difference
            #between initial and current headings
            thetaRad = math.radians(theta)
            diff = thetaRad - initialThetaRad

            #Adjust for values above pi radians and below -pi radians
            if(diff > math.pi):
                error = angle - (diff - 2*math.pi)
            elif(diff < -math.pi):
                error = angle - (diff + 2*math.pi)
            else:
                error = angle - diff

            #Rotate until desired heading is reacched
            if(abs(error) >= math.radians(2.0)):
                pub.publish(vel)
            else:
                done = True
                vel.angular.z = 0
                pub.publish(vel)

# This is the program's main function
def timerCallback(event):
    global pose
    global xPosition
    global yPosition
    global theta
    global map_list
    
    pose = Pose()
    
    map_list.waitForTransform('/map', '/base_link', rospy.Time.now(), rospy.Duration(0.1))
    if map_list.frameExists('/base_link') and map_list.frameExists('/map'):
        t = map_list.getLatestCommonTime('/map', '/base_link')
        (position, orientation) = map_list.lookupTransform('/map', '/base_link', t) #finds the position and oriention of two objects relative to each other (hint: this returns arrays, while Pose uses lists)
        pose.position.x = position[0]
        pose.position.y = position[1]
        xPosition = position[0]
        yPosition = position[1]

        odomW = orientation
        q = [odomW[0], odomW[1], odomW[2], odomW[3]]
        roll, pitch, yaw = euler_from_quaternion(q)
        pose.orientation.z = yaw
        theta = math.degrees(yaw)



#This is the program's main function
if __name__ == '__main__':
    #Change this node name to include your username
    rospy.init_node('msteidel_lab2_node')

    #These are global variables. Write "global <variable_name>" in any other function to gain access to these global variables 
    global pub
    global pose
    global map_tf
    global map_list

    pose = Pose()
    #Replace the elipses '...' in the following lines to set up the publishers and subscribers the lab requires
    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, None, queue_size=10) # Publisher for commanding robot motion
    #rviz_click = rospy.Subscriber('/goal', PoseStamped, navToPose, queue_size=1)
    path_sub = rospy.Subscriber('aStar_Path', Path, followPath, queue_size=1)
    #Use this object to get the robot's Odometry 
    map_list = tf.TransformListener()
    
    #Use this command to make the program wait for some seconds
    
    rospy.sleep(rospy.Duration(1, 0))
    timerCallback(1)

    #make the robot keep doing something...
    rospy.Timer(rospy.Duration(0.2), timerCallback)
    while(not rospy.is_shutdown()):
        rospy.sleep(0.15)
