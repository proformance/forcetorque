#!/usr/bin/env python

import rospy
import time
import tf
from numpy import matrix
from numpy import linalg
import math

import keyboard

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped, WrenchStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from keyhandlerown import getchar 
import os


#--------------------------------------------------------------------
class Commander():
    def __init__(self):
        
        # print(self.GraspingMatrix(matrix([[1],[2]])))
        
        # Start listener:
        # self.listener = tf.TransformListener()

        # Subscribe to robot  (/calibrated_fts_wrench)
        rospy.Subscriber("/joint_states", JointState, self.callback_joints)
        
        # Publish to robot
        self.urScriptPub = rospy.Publisher("/ur_driver/URScript", String, queue_size=1)

        # Go into spin, with rateOption!
        self.rate = rospy.Rate(10)          # 10hz
        rospy.loginfo(rospy.get_caller_id() + "Test started...")
        self.spin()

#CBs ----------------------------------------------------------------
#--------------------------------------------------------------------
    def callback_joints(self, data):
        self.joints = data.position;
        # print self.joints;
        # print type(self.joints)
        # print "--------------------------"

#     def callback_sensor(self, data):
#         self.sensor = data.
        
        
# spin --------------------------------------------------------------
    def spin(self):
        while (not rospy.is_shutdown()):
#             start_time = rospy.get_rostime() 
            # print time:
            # print("hello")
#             end = time.time()
            # print(end - start)
            # print '%d------' % (self.n1)

            # [rot,trans]=self.FTSframe()

            # print '---------------------dddddddddddddddddddd'
            # print rot
            # print trans
            # print '---------------------dddddddddddddddddddd'
            
            # command = "speedl([" +str(V_ref[0,0]) +","+  str(-V_ref[1,0]) +",0,0,0,"+ str(-V_ref[2,0]) + "],0.01, 0.1)";
            ch = getchar()
            if ch == 'w':
                command = "speedl([0,0,1,0,0,0],0.01, 0.5)"
                print 'up'
                self.urScriptPub.publish(command)
            elif ch == 's':
                command = "speedl([0,0,-1,0,0,0],0.01, 0.5)"
                print 'down'
                self.urScriptPub.publish(command)
            elif ch == 'q':
                rospy.signal_shutdown("q pressed")
                



            
            # Go into spin, with rateOption!
            self.rate.sleep()        

#--------------------------------------------------------------------
#     def FTSframe(self):  
#         try:			
#             self.listener.waitForTransform('/world', '/fts_link', rospy.Time(0),rospy.Duration(1))
#             (trans,rot) = self.listener.lookupTransform('/world', '/fts_link', rospy.Time(0))
#             # print trans
#             # print rot
#             R = tf.transformations.euler_from_quaternion(rot)
#             transrotM = self.listener.fromTranslationRotation(trans, rot)
#             # print transrotM
#             rotationMat = transrotM[0:3,0:3]
#             # print rotationMat
#             # print self.RotationMatrix(R)
#             ttt1 = rotationMat * matrix([[0],[0],[1]])
#             ttt2 = rotationMat.T * matrix([[0],[1],[0]])
#             print rotationMat
#             rot_matrix_z = matrix([[ math.cos(3.14), -math.sin(3.14) ,0], [  math.sin(3.14), math.cos(3.14), 0 ] , [ 0, 0 ,1] ])
#             print rot_matrix_z
#             print rot_matrix_z * rotationMat
#             # print self.RotationMatrix(R)
#             print "-------------------------------------------------"
#             # print R
#             return [R, trans]
#         except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#             print "EXCEPTION"
#             pass
            
# #--------------------------------------------------------------------    
#     def RotationMatrix(self, rot):
#         rot_matrix_x = matrix([[1, 0, 0 ], [0, math.cos(rot[0]), -math.sin(rot[0])], [ 0, math.sin(rot[0]), math.cos(rot[0])] ])
#         rot_matrix_y = matrix([[ math.cos(rot[1]), 0, math.sin(rot[1])],[0, 1, 0 ], [ -math.sin(rot[1]), 0, math.cos(rot[1])] ])
#         rot_matrix_z = matrix([[ math.cos(rot[2]), -math.sin(rot[2]) ,0], [  math.sin(rot[2]), math.cos(rot[2]), 0 ] , [ 0, 0 ,1] ])
#         rot_matrix = rot_matrix_z * rot_matrix_y * rot_matrix_x
#         return rot_matrix

# #--------------------------------------------------------------------    
#     def GraspingMatrix(self, Pg):
#         G_matrix = matrix([[1,0,0],[0,1,0],[-Pg[1,0],Pg[0,0],1]])
#         return G_matrix


            

#--------------------------------------------------------------------    
# Here is the main entry point
if __name__ == '__main__':
    try:
        # Init
        rospy.init_node('Project_Group_19')

        start = time.time()
        Commander()

        # Just keep the node alive!
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass
