#!/usr/bin/env python
# license removed for brevity
import rospy
import random,math
import numpy as np
import copy
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Char

pub = rospy.Publisher('CommandTopic', Char, queue_size=10)

class Reaching:
    def __init__(self):
        rospy.init_node('reaching', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        # self.model_pose = [0.0,0.2,0.2  , 0,0,0,1]
        self.model_pose = [0.0,0.3,0.1  , 0,0,0,1]
        self.obstacle_pose = [0.0,0.0,0.0]
        self.set_forbidden()
        self.set_init_pose()

    def set_forbidden(self):
        #set forbidden erea
        self.planning_scene = PlanningSceneInterface("base_link")
        self.planning_scene.removeCollisionObject("my_ground")
        self.planning_scene.addCube("my_ground", 2, 0, 0, -1.04)
        self.planning_scene.addBox("obstacle", 0.1, 0.5, 1, -0.3,0,0.5)
        self.planning_scene.addBox("obstacle2", 0.5, 0.1, 1, 0,-0.3,0.5)
        #self.planning_scene.addCube("demo_cube", 0.06,0,0.3,0)
        print dir(self.planning_scene)
        import inspect
        print "addBox's variable is ",inspect.getargspec(self.planning_scene.addBox)
        print "attachBox's variable is ",inspect.getargspec(self.planning_scene.attachBox)
        print "addCube's variable is ",inspect.getargspec(self.planning_scene.addCube)
        print "setColor's variable is ",inspect.getargspec(self.planning_scene.setColor)

    def set_init_pose(sefl): 
        #set init pose
        move_group = MoveGroupInterface("manipulator","base_link")
        planning_scene = PlanningSceneInterface("base_link")
        joint_names = ["shoulder_pan_joint", "shoulder_lift_joint",
                        "elbow_joint", "wrist_1_joint",
                        "wrist_2_joint", "wrist_3_joint"]
        pose = [-1.26,-0.64,-2.44,-0.66,1.56,0.007]      
        move_group.moveToJointPosition(joint_names, pose, wait=False)
        move_group.get_move_action().wait_for_result()
        result = move_group.get_move_action().get_result()
        move_group.get_move_action().cancel_all_goals()


    def callback(self,data):
        x = data.pose[1].position.x
        y = data.pose[1].position.y
        z = data.pose[1].position.z

    def start_subscriber(self):
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)

    def reaching_pose(self):
        print self.model_pose
        group = moveit_commander.MoveGroupCommander("manipulator")
        #print group.get_current_pose().pose
        pose_target = geometry_msgs.msg.Pose()
        pose = group.get_current_pose().pose
        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.x = self.model_pose[3]
        pose_target.orientation.y = self.model_pose[4]
        pose_target.orientation.z = self.model_pose[5]
        pose_target.orientation.w = self.model_pose[6]
        pose_target.position.x = self.model_pose[0]
        pose_target.position.y = self.model_pose[1]
        pose_target.position.z = self.model_pose[2]

        group.set_pose_target(pose_target)
        group.go()
        #raw_input("Enter -->")


    def gripper_close(self):
        print("Gripper_Close")
        gripper_pose = Char()
        gripper_pose = 255
        pub.publish(gripper_pose)
        #raw_input("Enter -->")

    
    def gripper_open(self):
        print("Gripper_Open")
        gripper_pose = Char()
        gripper_pose = 0
        pub.publish(gripper_pose)
        #raw_input("Enter -->")


    def move_45(self):
        rate = rospy.Rate(1)
        rospy.sleep(1)

        a = 0.065
        b = 0.040
        c = 0.012

        pattern_45 = [
                    [-0.10,0.31   ,0.015  , 0,0,0,1],   #B
                    'c',
                    [-0.10,0.31   ,0.1    , 0,0,0,1],   #B
                    #[0.0  ,0.31   ,0.15    , 0,0,0,1],  #A90
                    [0    ,0.5    ,0.081  , 0.3826834, 0, 0, 0.9238795],
                    [0    ,0.5    ,0.0365  , 0.3826834, 0, 0, 0.9238795],
                    'o',
                    [0    ,0.5    ,0.081  , 0.3826834, 0, 0, 0.9238795],
                    #[0.0  ,0.31   ,0.1    , 0,0,0,1],   #A90
                    [-0.10,0.401    ,0.041  , 0,0,0,1],
                    'c',
                    [-0.10,0.401    ,0.100  , 0,0,0,1],
                    [0    ,0.5-a  ,0.037+a, 0.3826834, 0, 0, 0.9238795],
                    [0    ,0.5-b  ,0.037+b, 0.3826834, 0, 0, 0.9238795],
                    [0    ,0.5-c  ,0.037+c, 0.3826834, 0, 0, 0.9238795],
                    'o',

                    'c',
                    [0    ,0.5-a  ,0.037+a, 0.3826834, 0, 0, 0.9238795],
                    [-0.10,0.401    ,0.100  , 0,0,0,1],
                    [-0.10,0.401    ,0.050  , 0,0,0,1],
                    'o',
                    [-0.10,0.401    ,0.100  , 0,0,0,1],
                    [0    ,0.5    ,0.081  , 0.3826834, 0, 0, 0.9238795],
                    [0    ,0.5    ,0.0365  , 0.3826834, 0, 0, 0.9238795],
                    'c',
                    [0    ,0.5    ,0.081  , 0.3826834, 0, 0, 0.9238795],
                    [-0.10,0.31   ,0.1    , 0,0,0,1],   #B
                    [-0.10,0.31   ,0.015  , 0,0,0,1],   #B
                    'o',
                    [-0.10,0.31   ,0.1    , 0,0,0,1],   #B
                    ]

        while not rospy.is_shutdown():
            for pose in pattern_45:
                if pose == 'c':
                    self.gripper_close()
                    rospy.sleep(1)
                elif pose == 'o':
                    self.gripper_open()
                    rospy.sleep(1)
                else :
                    self.model_pose = pose
                    self.reaching_pose()

                if rospy.is_shutdown():
                    return 0


    def move_90(self):
        rate = rospy.Rate(1)
        rospy.sleep(1)

        a = 0.065
        b = 0.040
        c = 0.012

        pattern_45 = [
                    [-0.10,0.31   ,0.015  , 0,0,0,1],   #B
                    'c',
                    [-0.10,0.31   ,0.1    , 0,0,0,1],   #B
                    [0.0  ,0.300   ,0.1    , 0,0,0,1],  #A90
                    [0.0  ,0.315  ,0.0358    , 0,0,0,1],  #A90
                    'o',
                    [0.0  ,0.315   ,0.1    , 0,0,0,1],  #A90
                    [-0.10,0.40    ,0.041  , 0,0,0,1], #negi
                    'c',
                    [-0.10,0.40    ,0.1    , 0,0,0,1],

                    [-0.001  ,0.316   ,0.1    , 0,0,0,1],  #A90
                    [-0.001  ,0.316   ,0.053    , 0,0,0,1],  #A90
                    'o',


                    'c',
                    [0.0  ,0.314   ,0.1    , 0,0,0,1],  #A90
                    [-0.10,0.40    ,0.1    , 0,0,0,1],
                    [-0.10,0.40    ,0.05  , 0,0,0,1], #negi
                    'o',
                    #[-0.10,0.40    ,0.1    , 0,0,0,1],
                    [0.0  ,0.300   ,0.1    , 0,0,0,1],  #A90
                    [0.0  ,0.315  ,0.036    , 0,0,0,1],  #A90
                    'c',
                    [0.0  ,0.300   ,0.1    , 0,0,0,1],  #A90
                    [-0.10,0.320   ,0.1    , 0,0,0,1],   #B
                    [-0.10,0.31   ,0.0153  , 0,0,0,1],   #B
                    'o',
                    [-0.10,0.31   ,0.1    , 0,0,0,1],   #B
                    ]

        while not rospy.is_shutdown():
            for pose in pattern_45:
                if pose == 'c':
                    self.gripper_close()
                    rospy.sleep(1)
                elif pose == 'o':
                    self.gripper_open()
                    rospy.sleep(1)
                else :
                    self.model_pose = pose
                    self.reaching_pose()

                if rospy.is_shutdown():
                    return 0





if __name__ == '__main__':
    try:
        r = Reaching()
        r.start_subscriber()
        r.move_90()
        
    except rospy.ROSInterruptException:
        pass