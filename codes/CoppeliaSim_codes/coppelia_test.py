import sim 
import numpy as np

def connect(ip='127.0.0.1', port=19999):
    sim.simxFinish(-1)#just in case, close all opened connections
    clientID = sim.simxStart(ip, port, True, True, 2000, 5) #Conection function
    if clientID == 0:
        print("Connected---{}:{}".format(ip, port))
    else:
        print("Error: connection refused")
    return clientID

clientID = connect()

#Get handler for a given object in Coppelia
returnCode, handle = sim.simxGetObjectHandle(clientID,'Dummy',sim.simx_opmode_blocking)
dummy_ = handle
print(dummy_)
input('Press enter to continue...')
#Function to get position of desired objects
returnCode, pos = sim.simxGetObjectPosition(clientID, dummy_, -1, sim.simx_opmode_blocking)
print(pos)
input('Press enter to continue...')
ret, joint1_ = sim.simxGetObjectHandle(clientID,'Joint1',sim.simx_opmode_blocking)
ret, joint2_ = sim.simxGetObjectHandle(clientID,'Joint2',sim.simx_opmode_blocking)
print(joint1_, joint2_)
input('Press enter to continue...')
returnCode, pos1 = sim.simxGetJointPosition(clientID, joint1_, sim.simx_opmode_blocking)
print(pos1)
input('Press enter to continue...')
returnCode, pos2 = sim.simxGetJointPosition(clientID, joint2_, sim.simx_opmode_blocking)
print(pos2)
input('Press enter to continue...')
#send a position to a joint in rads
q1 = -30 * np.pi/180
returnCode = sim.simxSetJointTargetPosition(clientID, joint1_, q1, sim.simx_opmode_oneshot)
print(returnCode)
input('Press enter to continue...')
q2 = 30 * np.pi/180
returnCode = sim.simxSetJointTargetPosition(clientID, joint2_, q1, sim.simx_opmode_oneshot)
print(returnCode)
input('Press enter to continue...')

q1 = -90 * np.pi/180
returnCode = sim.simxSetJointTargetPosition(clientID, joint1_, q1, sim.simx_opmode_oneshot)
q2 = 0 * np.pi/180
returnCode = sim.simxSetJointTargetPosition(clientID, joint2_, q1, sim.simx_opmode_oneshot)
returnCode, pos = sim.simxGetObjectPosition(clientID, dummy_, -1, sim.simx_opmode_blocking)
print(pos)
input('Press enter to continue...')