# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
import numpy as np
from math import atan2
import math
class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position
def astar(m,startp,endp):
    w,h = 20,20		# 10x10(blocks) is the dimension of the input images
    sx,sy = startp 	#Start Point
    ex,ey = endp 	#End Point
    #[parent node, x, y, g, f]
    node = [None,sx,sy,0,abs(ex-sx)+abs(ey-sy)] 
    closeList = [node]
    createdList = {}
    createdList[sy*w+sx] = node
    k=0
    while(closeList):
        node = closeList.pop(0)
        x = node[1]
        y = node[2]
        l = node[3]+1
        k+=1
        #find neighbours 
        if k!=0:
            neighbours = ((x,y+1),(x,y-1),(x+1,y),(x-1,y))
        else:
            neighbours = ((x+1,y),(x-1,y),(x,y+1),(x,y-1))
        for nx,ny in neighbours:
            if nx==ex and ny==ey:
                path = [(ex,ey)]
                while node:
                    path.append((node[1],node[2]))
                    node = node[0]
                return list(reversed(path))            
            if 0<=nx<w and 0<=ny<h and m[ny][nx]==0:
                if ny*w+nx not in createdList:
                    nn = (node,nx,ny,l,l+abs(nx-ex)+abs(ny-ey))
                    createdList[ny*w+nx] = nn
                    #adding to closelist ,using binary heap
                    nni = len(closeList)
                    closeList.append(nn)
                    while nni:
                        i = (nni-1)>>1
                        if closeList[i][4]>nn[4]:
                            closeList[i],closeList[nni] = nn,closeList[i]
                            nni = i
                        else:
                            break
    return []
m  = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0],
      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
      [0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
      [0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
      [0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
      [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
      [1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
      [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
      [0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
      [0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0],
      [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0]]


   
s_x = 2
s_y = 7

x = float(s_x / 2)
y = float(s_y / 2)

yaw_ = 0
state_=0

# parameters
yaw_precision_ = math.pi / 90 # +/- 2 degree allowed
dist_precision_ = 0.3

# publishers
pub = None

# callbacks
def clbk_odom(msg):
    global x
    global y
    global theta

    global yaw_
    
    # position
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    
    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]
def change_state(state):
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)

def fix_yaw(local_x, local_y):
    global yaw_, pub, yaw_precision_, state_, s_x,s_y
    s_x= local_x -x
    s_y = local_y - y
    desired_yaw = math.atan2(s_y, s_x)
    err_yaw = desired_yaw - yaw_
    
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_:
        twist_msg.angular.z = 0.7 if err_yaw > 0 else -0.7
    
    pub.publish(twist_msg)
    
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_:
        print ('Yaw error: [%s]' % err_yaw)
        change_state(1)

def go_straight_ahead(local_x, local_y):
    global yaw_, pub, yaw_precision_, state_,s_x,s_y
    s_x= local_x -x
    s_y = local_y - y

    desired_yaw = math.atan2(s_y, s_x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(s_y, 2) + pow(s_x, 2))
    
    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = -0.6
        pub.publish(twist_msg)
    else:
        print ('Position error: [%s]' % err_pos)
        change_state(2)
    
    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        print ('Yaw error: [%s]' % err_yaw)
        change_state(1)

def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)
 
def main():
    global pub
    
    rospy.init_node('go_to_point')
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    
    rate = rospy.Rate(20)
    goal=Point()
    goal.x=5
    goal.y=4.5
    goal.z=5
    g_x = int(goal.x * 2)
    g_y = int(goal.y * 2)
    start = (s_x, s_y)
    end = (g_x, g_y)
    path = astar(m,start,end)
    print(path)
    path = list(path)
    an_array = np.array(path)
    path = an_array * 0.5
    path = path[::-1]
    path = list(path)
    removed = path.pop()
    while path:
      if not path:
         command = "STOP"
      else:
        removed = path.pop()
      local_x = removed[0]
      local_y = removed[1]
      while not rospy.is_shutdown():
        if state_ == 0:
            fix_yaw(local_x, local_y)
        elif state_ == 1:
            go_straight_ahead(local_x, local_y)
        elif state_ == 2:
            done()
            pass
        else:
            rospy.logerr('Unknown state!')
            pass
        rate.sleep()
if __name__ == '__main__':
    main()          
