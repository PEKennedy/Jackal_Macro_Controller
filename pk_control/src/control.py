#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

# Inspired from http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
# Modified by Alexandre Vannobel to test the FollowJointTrajectory Action Server for the Kinova Gen3 robot

# To run this node in a given namespace with rosrun (for example 'my_gen3'), start a Kortex driver and then run :
# rosrun kortex_examples example_moveit_trajectories.py __ns:=my_gen3

import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_srvs.srv import Empty

from sensor_msgs.msg import Joy
import copy
from functools import partial #used to pass arguments in a callback

class ExampleMoveItTrajectories(object):
    escaped = False
    gripper_closed = False
    is_controller_present = True
    control_scheme = "Reference" # Reference or Primitives
    ref_mode = 0
    last_time = 0
    """ExampleMoveItTrajectories"""

    def __init__(self):

        # Initialize the node
        super(ExampleMoveItTrajectories, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('example_move_it_trajectories')

        try:
            self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
            if self.is_gripper_present:
                gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
                self.gripper_joint_name = gripper_joint_names[0]
            else:
                self.gripper_joint_name = ""
            self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

            # Create the MoveItInterface necessary objects
            arm_group_name = "arm"
            self.robot = moveit_commander.RobotCommander("robot_description")
            self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
            self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
            self.display_trajectory_publisher = rospy.Publisher(
                rospy.get_namespace() + 'move_group/display_planned_path',
                moveit_msgs.msg.DisplayTrajectory,
                queue_size=20)

            if self.is_gripper_present:
                gripper_group_name = "gripper"
                self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

            #self.is_controller_present = rospy.has_param("") #you can't check if a topic exists really
            """if self.is_controller_present:
                if self.control_scheme == "Primitives":
                    rospy.loginfo("Using Primitives control scheme")
                    rospy.Subscriber("joy", Joy, self.handle_controller_primitives)
                else:
                    rospy.loginfo("Using Reference control scheme")
                    rospy.Subscriber("joy", Joy, self.handle_controller_reference)
                rospy.loginfo("Using Controller")
            else:
                rospy.loginfo("Using Keyboard")
                # see https://answers.ros.org/question/295940/subscribe-to-actions-on-keyboard-press-key/
                # which links to https://github.com/ros-teleop/teleop_twist_keyboard/blob/master/teleop_twist_keyboard.py
                # for how to get keyboard started"""

            rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
        except Exception as e:
            print(e)
            self.is_init_success = False
        else:
            self.is_init_success = True

    def reach_named_position(self, target):
        arm_group = self.arm_group

        # Going to one of those targets
        rospy.loginfo("Going to named target " + target)
        # Set the target
        arm_group.set_named_target(target)
        # Plan the trajectory
        (success_flag, trajectory_message, planning_time, error_code) = arm_group.plan()
        # Execute the trajectory and block while it's not finished
        return arm_group.execute(trajectory_message, wait=True)

    def reach_joint_angles(self, tolerance):
        arm_group = self.arm_group
        success = True

        # Get the current joint positions
        joint_positions = arm_group.get_current_joint_values()
        rospy.loginfo("Printing current joint positions before movement :")
        for p in joint_positions: rospy.loginfo(p)

        # Set the goal joint tolerance
        self.arm_group.set_goal_joint_tolerance(tolerance)

        # Set the joint target configuration
        if self.degrees_of_freedom == 7:
            joint_positions[0] = pi / 2
            joint_positions[1] = 0
            joint_positions[2] = pi / 4
            joint_positions[3] = -pi / 4
            joint_positions[4] = 0
            joint_positions[5] = pi / 2
            joint_positions[6] = 0.2
        elif self.degrees_of_freedom == 6:
            joint_positions[0] = 0
            joint_positions[1] = 0
            joint_positions[2] = pi / 2
            joint_positions[3] = pi / 4
            joint_positions[4] = 0
            joint_positions[5] = pi / 2
        arm_group.set_joint_value_target(joint_positions)

        # Plan and execute in one command
        success &= arm_group.go(wait=True)

        # Show joint positions after movement
        new_joint_positions = arm_group.get_current_joint_values()
        rospy.loginfo("Printing current joint positions after movement :")
        for p in new_joint_positions: rospy.loginfo(p)
        return success

    def get_cartesian_pose(self):
        arm_group = self.arm_group

        # Get the current pose and display it
        pose = arm_group.get_current_pose()
        rospy.loginfo("Actual cartesian pose is : ")
        rospy.loginfo(pose.pose)

        return pose.pose

    def reach_cartesian_pose(self, pose, tolerance, constraints):
        arm_group = self.arm_group

        # Set the tolerance
        arm_group.set_goal_position_tolerance(tolerance)

        # Set the trajectory constraint if one is specified
        if constraints is not None:
            arm_group.set_path_constraints(constraints)

        # Get the current Cartesian Position
        arm_group.set_pose_target(pose)

        # Plan and execute
        rospy.loginfo("Planning and going to the Cartesian Pose")
        return arm_group.go(wait=True)

    def reach_gripper_position(self, relative_position):
        gripper_group = self.gripper_group

        # We only have to move this joint because all others are mimic!
        gripper_joint = self.robot.get_joint(self.gripper_joint_name)
        gripper_max_absolute_pos = gripper_joint.max_bound()
        gripper_min_absolute_pos = gripper_joint.min_bound()
        try:
            rospy.loginfo(
                relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos)
            val = gripper_joint.move(
                relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos,
                True)
            return val
        except:
            return False

    # https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html#getting-started
    # Under Cartesian paths, demonstrates moving along waypoints
    def follow_path_ex(self):
        waypoints = []

        # wpose = move_group.get_current_pose().pose
        wpose = self.get_cartesian_pose()
        wpose.position.z -=  0.1  # First move up (z)
        wpose.position.y +=  0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x +=  0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -=  0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.arm_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold
        return plan, fraction

            # Xbox button mapping
            # 0==A, 1==B, 2==X, 3==Y
            # 4==LB  5==RB
            # stickBtnL== 9, STBR == 10
            # Start Btn ==7 , select==6, Xbox==8

            # axes:
            # dpad X,Y == 6,7 (Y inverted)
            # L stick X,Y ==0,1 (Y inverted)
            # R stick X,Y ==3,4 (Y inverted)
            # LT==2 , RT==5

            #PS axes are same, some button diffs
            # 2==Triangle, 3==square
            # stick btns are 11 (L) and 12 (R)
            # 8 9 10 =>share, options, ps btn
            # 6,7 are triggers all the way down

    #Future improvement: instead just track what the last button was and its count rather than an array
    #this also means you can't get a double press by rocking back btw 2 btns
    #or just have a mechanism to cancel if we switch btns, but for now this is good enough
    button_ups = [0,0,0,0]
    lastButtonDown = [False,False,False,False]
    buttonDown = [False,False,False,False]
    onButtonUp = [False,False,False,False]
    def double_pressed(self, btnIndex, event):
        if self.button_ups[btnIndex] > 1:
            rospy.loginfo("Double Pressed")
            rospy.loginfo(btnIndex)
            rospy.loginfo("##############")
        else:
            rospy.loginfo("Single Pressed")
            rospy.loginfo(btnIndex)
            rospy.loginfo("##############")
            #single press B for a gripper toggle
            if btnIndex == 1:
                self.toggle_gripper()
        self.button_ups[btnIndex] = 0


    #export ROS_MASTER_URI=http://localhost:11311, unset ROS_IP for each terminal
    #launch each of these in their own terminal
    #roscore
    #roslaunch kortex_gazebo spawn_kortex_robot.launch arm:=gen3_lite
    #rosparam set joy_node/dev "/dev/input/jsX(2), rosrun joy joy_node
    #roslaunch pk_control control.launch

    def check_press(self, btnIndex, btnState):
        #rospy.loginfo("Do Check Press")
        self.lastButtonDown[btnIndex] = self.buttonDown[btnIndex]
        if btnState == 1:
            self.buttonDown[btnIndex] = True
        else:
            self.buttonDown[btnIndex] = False

        self.onButtonUp[btnIndex] = self.lastButtonDown[btnIndex] and not self.buttonDown[btnIndex]

        if self.onButtonUp[btnIndex]:
            if self.button_ups[btnIndex] == 0:
                pressedCallback = partial(self.double_pressed,btnIndex) #make a callback "partial" func which has btnIndex filled out
                rospy.Timer(rospy.Duration(0.2), pressedCallback, oneshot=True)
            self.button_ups[btnIndex] += 1
            rospy.loginfo(self.button_ups[btnIndex])



        #if button up
            #timeout for 200 milis
            #if timed out >> single press
            #elsif button up again >> clear timeout, double pressed


    def handle_controller_primitives(self, data):
        # pprint(vars(data))
        # rospy.loginfo(data)
        #rospy.loginfo("Hello there")
        #rospy.loginfo(data)
        btnA = data.buttons[0]
        btnB = data.buttons[1]
        btnEscape = data.buttons[8] #share on ps, xbox btn on xbox
        ax0 = data.axes[0]

        self.check_press(0, btnA)
        self.check_press(1, btnB)

        if btnEscape == 1:
            rospy.loginfo("Escaped")
            self.escaped = True

        #if btnB == 1:
        #    self.toggle_gripper()
        # rospy.loginfo("btnA is: " + str(btnA) + " ax0 is" + str(ax0))

    def toggle_gripper(self):
        rospy.loginfo("CALLED TOGGLE GRIPPER")
        if self.gripper_closed:
            # self.example_send_gripper_command(1.0)
            self.reach_gripper_position(1.0)
            self.gripper_closed = False
            rospy.loginfo("open")
        else:
            # self.example_send_gripper_command(0.0)
            self.reach_gripper_position(0)
            self.gripper_closed = True
            rospy.loginfo("close")

    # cycle modes
    def change_mode(self):
        if self.control_scheme == "Reference":
            self.ref_mode = (self.ref_mode + 1) % 3

    #Let Z be in and out, X side to side, Y be up and down
    def handle_controller_reference(self, data):
        cur_time = time.time()
        dt = cur_time - self.last_time
        #rospy.loginfo("Called handle controller")
        #rospy.loginfo(data)
        ax0 = data.axes[0]
        ax1 = data.axes[1]
        btnA = data.buttons[0]
        btnB = data.buttons[1]

       # rospy.loginfo("##### ###### #####")
       # rospy.loginfo(btnB)
        # X-Z movement
        if self.ref_mode == 0:
            pose_to = self.get_cartesian_pose()
            pose_to.position.z += -2*ax1*dt
            pose_to.position.x += 2*ax0*dt
            self.reach_cartesian_pose(pose=pose_to, tolerance=0.01, constraints=None)

        # X-Y movement
        #elif self.ref_mode == 1:

        # Gripper Twist and final joint's angle
        #else:


        if btnA == 1:
            rospy.loginfo("Escaped")
            self.escaped = True

        if btnB == 1:
            rospy.loginfo("%%%%%%%%%%%%%%%%%%%%%%")
            self.toggle_gripper()
        self.last_time = cur_time

    def main(self):
        example = self #ExampleMoveItTrajectories()

        # For testing purposes
        success = example.is_init_success
        rospy.loginfo(success)
        try:
            rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
        except:
            pass

        if example.is_controller_present:
            #if example.control_scheme == "Primitives":
                rospy.loginfo("Using Primitives control scheme")
                # queue_size=1 means no input buffering, only latest command
                # this means inputs can be eaten, but we aren't queueing ancient ones either
                rospy.Subscriber("/joy", Joy, example.handle_controller_primitives, queue_size=3)
            #else:
            #    rospy.loginfo("Using Reference control scheme")
            #    rospy.Subscriber("/joy", Joy, self.handle_controller_reference, queue_size=3)
            #self.last_time = time.time()
            #rospy.loginfo("Using Controller")
        else:
            #rospy.loginfo("Using Keyboard")
            rospy.loginfo("No controller")

        if success:
            rospy.loginfo("Reaching Named Target Home...")
            success &= example.reach_named_position("home")
            print(success)

        if example.is_gripper_present and success:

            """Our gripper test"""
            if example.is_controller_present:
                rospy.loginfo("Control gripper with controller")
                example.escaped = False

                i = 0
                while not example.escaped:
                    #rospy.loginfo("iter %i", i)
                    # rospy.loginfo(self.escaped)
                    rospy.sleep(0.01)
                    i += 1
            else:
                rospy.loginfo("demo gripper")

                #pose_to = example.get_cartesian_pose()
                #pose_to.position.z += 0.2
                #success &= example.reach_cartesian_pose(pose=pose_to, tolerance=0.01, constraints=None)
                #print(success)

                rospy.loginfo("do waypoint example")
                plan, x = example.follow_path_ex()
                example.arm_group.execute(plan, wait=True)

                rospy.loginfo("Opening the gripper...")
                success &= example.reach_gripper_position(0)
                print(success)

                rospy.loginfo("Closing the gripper 50%...")
                success &= example.reach_gripper_position(0.5)
                print(success)

        # For testing purposes
        rospy.set_param("/kortex_examples_test_results/moveit_general_python", success)

        if not success:
            rospy.logerr("The example encountered an error.")
        else:
            rospy.loginfo("Finished session")

if __name__ == '__main__':
    #main()
    ex = ExampleMoveItTrajectories()
    ex.main()



    """
  if success:
    rospy.loginfo("Reaching Named Target Vertical...")
    success &= example.reach_named_position("vertical")
    print (success)

  if success:
    rospy.loginfo("Reaching Joint Angles...")  
    success &= example.reach_joint_angles(tolerance=0.01) #rad
    print (success)

  if success:
    rospy.loginfo("Reaching Named Target Home...")
    success &= example.reach_named_position("home")
    print (success)

  if success:
    rospy.loginfo("Reaching Cartesian Pose...")

    actual_pose = example.get_cartesian_pose()
    actual_pose.position.z -= 0.2
    success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)
    print (success)

  if example.degrees_of_freedom == 7 and success:
    rospy.loginfo("Reach Cartesian Pose with constraints...")
    # Get actual pose
    actual_pose = example.get_cartesian_pose()
    actual_pose.position.y -= 0.3

    # Orientation constraint (we want the end effector to stay the same orientation)
    constraints = moveit_msgs.msg.Constraints()
    orientation_constraint = moveit_msgs.msg.OrientationConstraint()
    orientation_constraint.orientation = actual_pose.orientation
    constraints.orientation_constraints.append(orientation_constraint)

    # Send the goal
    success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=constraints)
    """