#!/usr/bin/env python
###############################################
# This python program is a bidirectional 
# interface between ROS and a Panasonic 
# VR-006L controller. In this script
# the user can send cartesian poses 
# to the robot through an interface in 
# the terminal. The specified movement can 
# be previewed in MoveIt! 
# The connection is established over the 
# Profibus network by using an open implmentation
# of this fieldbus protocol.
#          v--------------v-----------v
#          |   Panasonic  |    ROS    |
#          |     GII      |   Linux   |
#          |  Controller  |           |
#          v--------------v-----------v
#          |     DP       |    DP     |
#          |    Slave     |   Master  |
#          ^--------------^-----------^
#
###############################################

###############################################
#              import libraries               #
###############################################

import sys, threading, math, copy, array, time
from time import sleep
import numpy as np
import rospy
import pyprofibus
from pyprofibus import DpTelegram_SetPrm_Req, monotonic_time
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose
import visualization_msgs.msg 
from visualization_msgs.msg import Marker


###########################################

'''
The function getInfoFromTerminal() gets the user input from the terminal
and checks if the input is in between the safety limits of the robot. These
limits can be increased depending on the available robot working space. The 
user can specify a desired cartesian pose and can choose between a point-
to-point, a linear or a circular movement between start and goal. The function
returns the x,y,z position and orientation and the specified movement as integers.
'''

def getInfoFromTerminal():
    while True:
        try:
            x = int(input("Give an integer value for the x position in mm between 100 and 1400\n(home x position is 1120mm) :"))
        except ValueError:
            print("Sorry, I didn't understand that.")
        if not 100 <= x <= 1400:
            print("Sorry, x must be between 100 and 1400.")
            continue
        else:
            break
    while True:
        try:
            y = int(input("Give an integer value for the y position in mm between -900 and 900\n(home y position is 0mm):"))
        except ValueError:
            print("Sorry, I didn't understand that.")
        if not -900 <= y <= 900:
            print("Sorry, y must be between -900 and 900.")
            continue
        else:
            break
    while True:
        try:
            z = int(input("Give an integer value for the z position in mm between 0 and 1400\n(home z position is 725mm):"))
        except ValueError:
            print("Sorry, I didn't understand that.")
        if not 0 <= z <= 1400:
            print("Sorry, z must be between 0 and 1400.")
            continue
        else:
            break   
    while True:
        try:
            Rx = int(input("Give an integer value for the x orientation in degrees between -180 and 180\n(home Rx orientation is 0):"))
        except ValueError:
            print("Sorry, I didn't understand that.")
        if not -180 <= Rx <= 180:
            print("Sorry, Rx must be between -180 and 180.")
            continue
        else:
            break
    while True:
        try:
            Ry = int(input("Give an integer value for the y orientation in degrees between -180 and 180\n(home Ry orientation is 45):"))
        except ValueError:
            print("Sorry, I didn't understand that.")
        if not -180 <= Ry <= 180:
            print("Sorry, Ry must be between -180 and 180.")
            continue
        else:
            break
    while True:
        try:
            Rz = int(input("Give an integer value for the z orientation in degrees between -180 and 180\n(home Rz orientation is 0):"))
        except ValueError:
            print("Sorry, I didn't understand that.")
        if not -180 <= Rz <= 180:
            print("Sorry, Rz must be between -180 and 180.")
            continue
        else:
            break
    while True:
        try:
            mType = int(input("Now give a movetype: Point-to-point (1), Continuous-Linear (2), Continuous-Circular (3) --> "))
        except ValueError:
            print("Sorry, I didn't understand that.")
            continue
        if not (mType == 1 or mType == 2 or mType == 3):
            print("Sorry, movetype must be 1 or 2 or 3.")
            continue
        else:
            break
    
    return x, y, z, Rx, Ry, Rz, mType


'''
The function convertValueFirstByte has as argument an integer value,
converts this to a binary representation and returns the first byte of
this integer value.
'''

def convertValueFirstByte(value):
    
    convertedValue = int('{0:016b}'.format(((1<<16) -1) & value)[8:],2) 

    return convertedValue


'''
The function convertValueSecondByte has as argument an integer value,
converts this to a binary representation and returns the second byte of
this integer value.
'''

def convertValueSecondByte(value):
    
    convertedValue = int('{0:016b}'.format(((1<<16) -1) & value)[:8],2) 

    return convertedValue

'''
The function convertMovementType has as argument the specified movement type
value that the user specified before and returns its binary representation.
'''

def convertMovementType(value):

    convertedValue = int('{0:08b}'.format(value),2)

    return convertedValue

'''
The function convertFlag converts the number '1' to a binary representation
and returns this binary representation. The flag is used in the program to 
have a robust way to communicate with the robot. If the flag isn't set, the
robot will not execute its imposed movement.
'''

def convertFlag():

    convertedValue = int('{0:08b}'.format(1),2) 

    return convertedValue

'''
The function getValues uses the functions described above to return an
array of 14 bytes to prepare it to send to the robot when in data exchange
mode. The common I/O travel between ROS and robot controller is specified as
14 bytes (= 112 I/O's). It also returns an array of seven integer numbers that 
are used to visualise the specified movement in RViz (with MoveIt!).
'''

def getValues():
    data = getInfoFromTerminal()
    viz_array = data

    x1 = convertValueFirstByte(data[0])
    x2 = convertValueSecondByte(data[0])

    y1 = convertValueFirstByte(data[1])
    y2 = convertValueSecondByte(data[1])

    z1 = convertValueFirstByte(data[2])
    z2 = convertValueSecondByte(data[2])

    Rx1 = convertValueFirstByte(data[3])
    Rx2 = convertValueSecondByte(data[3])

    Ry1 = convertValueFirstByte(data[4])
    Ry2 = convertValueSecondByte(data[4])

    Rz1 = convertValueFirstByte(data[5])
    Rz2 = convertValueSecondByte(data[5])

    flag = convertFlag()
    mType = convertMovementType(data[6])

    posRotArray = [x1, x2, y1, y2, z1, z2, Rx1, Rx2, Ry1, Ry2, Rz1, Rz2, flag, mType]
    

    return posRotArray, viz_array  


'''
The function setupCommunication sets up the connection between ROS and the robot
controller over the PROFIBUS network. This function relies  on a config-file, which
can be found in the config folder. Here, the configuration of the slave with the master
is implemented. When the configuration is finished, the slave is added to the master-slave
network. If the slave doesn't react or the connection is lost, a PROFIBUS error will occur
and the configuration has to be set again.
'''

def setupCommunication():

    try:
        config  = pyprofibus.PbConf.fromFile("/home/david/robot_ws/src/vr_driver/config/panaprofi.conf")
        
        phy = config.makePhy()

        master = pyprofibus.DPM1(phy=phy, masterAddr=config.dpMasterAddr, debug=False)

        for slaveConf in config.slaveConfs:
            gsd = slaveConf.gsd

            slaveDesc = pyprofibus.DpSlaveDesc(identNumber=gsd.getIdentNumber(), slaveAddr=slaveConf.addr)
            slaveDesc.setCfgDataElements(gsd.getCfgDataElements())
            slaveDesc.setSyncMode(slaveConf.syncMode)
            slaveDesc.setFreezeMode(slaveConf.freezeMode)
            slaveDesc.setGroupMask(slaveConf.groupMask)
            slaveDesc.setWatchdog(slaveConf.watchdogMs)

        master.addSlave(slaveDesc)

        master.initialize()
        return master, slaveDesc

    except pyprofibus.ProfibusError as e:
        print("Terminating: %s" % str(e))
        rospy.logerr("Communication is lost. Try to initialize again")

        return None, None

'''
The function sendValues uses the information of the setupCommunication function to
send the Cartesian pose and specified movement type to the robot. The function returns
the output that the slave sends to the master in data exchange. The output consists of 
14 bytes. The first 6 bytes of this array contains the Cartesian position of the robot
when it has reached his imposed Cartesian pose.
'''

def sendValues(master, slaveDesc, dataArray):
    outputArray = None
    while (outputArray is None):
        outputArray = master.runSlave(slaveDesc, dataArray)

    return outputArray



'''
The function deformatOutput() has as argument the output that the slave send to the
master and return an integer number that respresents the Cartesian position of the
robot: [X,Y,Z]^{T}. This representation is needed to visualise the outcome in RViz.
'''
def deformatOutput(newArray):
    x1 = '{0:08b}'.format(newArray[0])
    x2 = '{0:08b}'.format(newArray[1])
    y1 = '{0:08b}'.format(newArray[2])
    y2 = '{0:08b}'.format(newArray[3])
    z1 = '{0:08b}'.format(newArray[4])
    z2 = '{0:08b}'.format(newArray[5])

    X = int(x1+x2,2)
    Y = int(y1+y2,2)
    Z = int(z1+z2,2)

    return [X, Y, Z]


'''
The function visualiseMovement() has as argument the array of integer numbers that
corresponds to the Cartesian position and orientation of a 3D point. MoveIt! shows the 
movement in simulation together with RViz. 
'''

def visualiseMovement(viz_array):
    
    moveit_commander.roscpp_initialize(sys.argv)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = float(viz_array[0])/(1000*2.604)      # convert to meter for ROS
    pose_target.position.y = float(viz_array[1])/(1000*2.5)
    pose_target.position.z = float(viz_array[2])/(1000*2.487)

    pi = math.pi
    pose_target.orientation.x = (float(viz_array[3])*pi)/180
    pose_target.orientation.y = (float(viz_array[4])*pi)/180 - pi/4
    print pose_target.orientation.y
    pose_target.orientation.z = (float(viz_array[5])*pi)/180

    group.set_pose_target(pose_target)

    plan = group.plan()
    
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory_publisher.publish(display_trajectory)
    group.execute(plan)

    moveit_commander.roscpp_shutdown()


'''
The function setMarker() uses the data that the robot sends to ROS to set a Marker
in RViz. This Marker specifies the goal position of the robot's movement when the 
robot has executed his trajectory and reported it to ROS.
'''

def setMarker(viz_array):
    
    start = time.time()

    period = 5

    while not rospy.is_shutdown():

        topic = 'visualization_marker'
        marker_publisher = rospy.Publisher(topic, Marker, queue_size=10)

        sphere_marker = Marker()
        sphere_marker.header.frame_id = "/base"

        sphere_marker.type = Marker.SPHERE
        sphere_marker.action = Marker.ADD

        #sphere_marker.ns = "my_marker"
        #sphere_marker.id = 0
        
        sphere_marker.scale.x = 0.05
        sphere_marker.scale.y = 0.05
        sphere_marker.scale.z = 0.05

        sphere_marker.color.a = 1.0
        sphere_marker.color.r = 0.0
        sphere_marker.color.g = 0.0
        sphere_marker.color.b = 1.0

        sphere_marker.pose.position.x = float(viz_array[0])/(1000*2.604)
        sphere_marker.pose.position.y = float(viz_array[1])/1000
        sphere_marker.pose.position.z = float(viz_array[2])/(1000*2.487)
        sphere_marker.pose.orientation.w = 1.0

        marker_publisher.publish(sphere_marker)

        if time.time() > start + period:
            break



'''
The function main starts the ROS interface node that interacts with the robot controller. 
First it starts the ROS node. Next it sets up the communication with the robot controller.
The user input is used to split up the amount of data. Afterwards communication with the 
robot can be contracted.
'''    

def main():
    try:
        rospy.init_node('interface', anonymous=True)
        
        rospy.loginfo("Initializing position streaming interface")
        
        rospy.loginfo("Setting up communication with Panasonic robot controller")
        
        master, slaveDesc = setupCommunication()

                
        if master is not None:

            while True:
            
                rospy.loginfo("Getting user information from terminal")

                dataArray, viz_array = getValues()
                visualiseMovement(viz_array)            # comment this function if visualisation isn't necessary
                setMarker(viz_array)                
                rospy.loginfo("Sending position and orientation to robot")
                
                outputArray = sendValues(master, slaveDesc, dataArray)
               
                
                if outputArray[1] != 0 or outputArray[2] != 0 or outputArray[3] != 0 or outputArray[4] != 0 or outputArray[5] != 0:
                    newArray = outputArray
                    pos_robot = deformatOutput(newArray)
                    setMarker(pos_robot)                # set marker at Cartesian point in RViz when robot has reached desired pose

                    rospy.loginfo("Robot sent feedback of actual Cartesian position")

                rospy.spin()


    except KeyboardInterrupt:
        rospy.logerr("Keyboardinterrupt")
        rospy.signal_shutdown("KeyboardInterrupt")
        raise


if __name__ == '__main__': main()
