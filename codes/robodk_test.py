from robolink import *
from robodk import *
import numpy as np
import math

def FromXYZRPWToArray(xyzrpw):
    if len(xyzrpw) < 6:
        return null
    x,y,z,r,p,w = xyzrpw  
    a = r * math.pi / 180.0
    b = p * math.pi / 180.0
    c = w * math.pi / 180.0
    ca = np.cos(a)
    sa = np.sin(a)
    cb = np.cos(b)
    sb = np.sin(b)
    cc = np.cos(c)
    sc = np.sin(c)
    return np.array([[cb * cc, cc * sa * sb - ca * sc, sa * sc + ca * cc * sb, x], 
                    [cb * sc, ca * cc + sa * sb * sc, ca * sb * sc - cc * sa, y], 
                    [-sb, cb * sa, ca * cb, z], 
                    [0., 0., 0., 1.]]);

def FromURToArray(xyzwpr):
    if len(xyzwpr) < 6:
        return null
    x,y,z,w,p,r = xyzwpr  
    angle = math.sqrt(w * w + p * p + r * r)
    if (angle < 1e-6):
        return np.identity(4)
    c = np.cos(angle)
    s = np.sin(angle)
    ux = w / angle
    uy = p / angle
    uz = r / angle
    return np.array([[ux * ux + c * (1 - ux * ux), ux * uy * (1 - c) - uz * s, ux * uz * (1 - c) + uy * s, x],
                [ux * uy * (1 - c) + uz * s, uy * uy + (1 - uy * uy) * c, uy * uz * (1 - c) - ux * s, y],
                [ux * uz * (1 - c) - uy * s, uy * uz * (1 - c) + ux * s, uz * uz + (1 - uz * uz) * c, z],
                [0., 0., 0., 1.]]);
# Connect to the RoboDK API
RDK = Robolink()
print("Connection test: \n")
if RDK.Connect():
    print("Connection test successful!")
else:
    print("There was a problem with the connection.\n Check if RoboDK is running, also check IP and remote API configuration\n")
    exit()
#print(RDK.Connect())
# Retrieve all items and print their names
list_items = RDK.ItemList()
print("Items in RoboDk station: ")
for item in list_items:
    print(item.Name())   

print("\nRetrieving robot... \n")
robot = RDK.Item('UR10e')      # retrieve the robot by name
if robot.Type() != 2:
    print("The selected item is not a robot")
else:
    print("Robot is: {} \n".format(robot.Name()))

print("Moving robot to default position\n")
robot.setJoints([0, -90, -90, 0, 90, 0])   # set the robot to the default configuration

# get the current robot joints
robot_joints = robot.Joints().tolist()
print("Current robot joints positions:")
print(robot_joints)
# get the robot position from the joints (calculate forward kinematics)
robot_position = robot.SolveFK(robot_joints)
print("\nCurrent robot tool position (forward kinematics with respect to base frame; ")
print("Pose: position (mm) and Euler angles (deg)):")
print(robot_position)
# get the robot configuration (robot joint state)
robot_config = robot.JointsConfig(robot_joints)
print("\nCurrent robot configuration state (defined as: [REAR, LOWERARM, FLIP]): ")
print(robot_config)
input('continue: [y/n]')
target = robot.Pose()             # retrieve the current target as a pose (position of the active tool with respect to the active reference frame)
#print(target)
xyzuvw = Pose_2_UR(target)        # Convert the 4x4 pose matrix to XYZUVW position and orientation angles (mm and rad)
print(xyzuvw)
print(FromURToArray(xyzuvw))
x,y,z,u,v,w = xyzuvw                # Calculate a new pose based on the previous pose
xyzuvw2 = [x,y+174.5,z*1.2,u,v,w]
target2 = UR_2_Pose(xyzuvw2)      # Convert the XYZABC array to a pose (4x4 matrix)
print(target2)
robot.MoveJ(target2)                # Make a linear move to the calculated position

tic()
while True:
    time = toc()
    print('Current time (s):' + str(time))
    joints = str(robot.Joints().tolist())
    #target = robot.Pose()               # retrieve the current target as a pose (position of the active tool with respect to the active reference frame)
    #xyzrpw = Pose_2_UR(target)        # Convert the 4x4 pose matrix to XYZRPW position and orientation angles (mm and rad)
    #print(FromURToArray(xyzrpw))
    print(str(time) + ', ' + joints[1:-1] + '\n')
    pause(1)