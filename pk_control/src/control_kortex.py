#!/usr/bin/env python3

import sys
import rospy
import time
from kortex_driver.srv import *
from kortex_driver.msg import *

# import controller for example like https://andrewdai.co/xbox-controller-ros.html#rosjoy
from sensor_msgs.msg import Joy
import copy
from functools import partial #used to pass arguments in a callback

# debug
# from pprint import pprint


# derived from the ExampleFullArmMove python class

class Control:
    escaped = False
    gripper_closed = False

    def __init__(self):
        try:
            rospy.init_node('example_full_arm_movement_python')

            self.HOME_ACTION_IDENTIFIER = 2

            # Get node params
            self.robot_name = rospy.get_param('~robot_name', "my_gen3_lite")
            self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 7)
            self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", False)

            rospy.loginfo("Using robot_name " + self.robot_name + " , robot has " + str(
                self.degrees_of_freedom) + " degrees of freedom and is_gripper_present is " + str(
                self.is_gripper_present))

            # Init the action topic subscriber
            self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification,
                                                     self.cb_action_topic)
            self.last_action_notif_type = None

            rospy.loginfo("A")

            # Init the services
            clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

            rospy.loginfo("B")

            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

            rospy.loginfo("C")

            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            rospy.loginfo("D")

            set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
            rospy.wait_for_service(set_cartesian_reference_frame_full_name)
            self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name,
                                                                    SetCartesianReferenceFrame)

            rospy.loginfo("E")

            play_cartesian_trajectory_full_name = '/' + self.robot_name + '/base/play_cartesian_trajectory'
            rospy.wait_for_service(play_cartesian_trajectory_full_name)
            self.play_cartesian_trajectory = rospy.ServiceProxy(play_cartesian_trajectory_full_name,
                                                                PlayCartesianTrajectory)

            rospy.loginfo("F")

            play_joint_trajectory_full_name = '/' + self.robot_name + '/base/play_joint_trajectory'
            rospy.wait_for_service(play_joint_trajectory_full_name)
            self.play_joint_trajectory = rospy.ServiceProxy(play_joint_trajectory_full_name, PlayJointTrajectory)

            rospy.loginfo("G")

            send_gripper_command_full_name = '/' + self.robot_name + '/base/send_gripper_command'
            rospy.wait_for_service(send_gripper_command_full_name)
            self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

            rospy.loginfo("H")

            activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(
                activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)

            rospy.loginfo("I")

        except:
            self.is_init_success = False
        else:
            self.is_init_success = True

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event

    def wait_for_action_start_n_finish(self):
        while not rospy.is_shutdown():
            if(self.last_action_notif_type == ActionEvent.ACTION_START):
                rospy.loginfo("Received ACTION_START notification")
                return self.wait_for_action_end_or_abort()

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                return False
            else:
                time.sleep(0.01)

    def example_subscribe_to_a_robot_notification(self):
        # Activate the publishing of the ActionNotification
        #rospy.loginfo("xyz??")
        req = OnNotificationActionTopicRequest()
        rospy.loginfo("Activating the action notifications...")
        try:
            self.activate_publishing_of_action_notification(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call OnNotificationActionTopic")
            return False
        else:
            rospy.loginfo("Successfully activated the Action Notifications!")

        rospy.sleep(1.0)
        return True

    def example_clear_faults(self):
        try:
            rospy.loginfo("Clear Faults....")
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)
            return True

    def example_home_the_robot(self):
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        self.last_action_notif_type = None
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False
            else:
                return self.wait_for_action_end_or_abort()

    def example_set_cartesian_reference_frame(self):
        self.last_action_notif_type = None
        # Prepare the request with the frame we want to set
        req = SetCartesianReferenceFrameRequest()
        req.input.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED

        # Call the service
        try:
            self.set_cartesian_reference_frame()
        except rospy.ServiceException:
            rospy.logerr("Failed to call SetCartesianReferenceFrame")
            return False
        else:
            rospy.loginfo("Set the cartesian reference frame successfully")

        # Wait a bit
        rospy.sleep(0.25)
        return True

    """def example_send_cartesian_pose(self):
        self.last_action_notif_type = None
        # Get the actual cartesian pose to increment it
        # You can create a subscriber to listen to the base_feedback
        # Here we only need the latest message in the topic though
        feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)

        req = PlayCartesianTrajectoryRequest()
        req.input.target_pose.x = feedback.base.commanded_tool_pose_x
        req.input.target_pose.y = feedback.base.commanded_tool_pose_y
        req.input.target_pose.z = feedback.base.commanded_tool_pose_z + 0.10
        req.input.target_pose.theta_x = feedback.base.commanded_tool_pose_theta_x
        req.input.target_pose.theta_y = feedback.base.commanded_tool_pose_theta_y
        req.input.target_pose.theta_z = feedback.base.commanded_tool_pose_theta_z

        pose_speed = CartesianSpeed()
        pose_speed.translation = 0.1
        pose_speed.orientation = 15

        # The constraint is a one_of in Protobuf. The one_of concept does not exist in ROS
        # To specify a one_of, create it and put it in the appropriate list of the oneof_type member of the ROS object :
        req.input.constraint.oneof_type.speed.append(pose_speed)

        # Call the service
        rospy.loginfo("Sending the robot to the cartesian pose...")
        try:
            self.play_cartesian_trajectory(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call PlayCartesianTrajectory")
            return False
        else:
            return self.wait_for_action_end_or_abort()"""

    #example which as I understand will work on the real robot, but not in simulation
    def example_send_cartesian_pose(self):
        self.last_action_notif_type = None
        # Get the actual cartesian pose to increment it
        # You can create a subscriber to listen to the base_feedback
        # Here we only need the latest message in the topic though
        feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)

        # Possible to execute waypointList via execute_action service or use execute_waypoint_trajectory service directly
        req = ExecuteActionRequest()
        trajectory = WaypointList()

        trajectory.waypoints.append(
            self.FillCartesianWaypoint(
                feedback.base.commanded_tool_pose_x,
                feedback.base.commanded_tool_pose_y,
                feedback.base.commanded_tool_pose_z + 0.10,
                feedback.base.commanded_tool_pose_theta_x,
                feedback.base.commanded_tool_pose_theta_y,
                feedback.base.commanded_tool_pose_theta_z,
                0)
        )

        trajectory.duration = 0
        trajectory.use_optimal_blending = False

        req.input.oneof_action_parameters.execute_waypoint_list.append(trajectory)

        # Call the service
        rospy.loginfo("Sending the robot to the cartesian pose...")
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ExecuteWaypointTrajectory")
            return False
        else:
            return self.wait_for_action_end_or_abort()


    def example_send_joint_angles(self):
        self.last_action_notif_type = None
        # Create the list of angles
        req = PlayJointTrajectoryRequest()
        # Here the arm is vertical (all zeros)
        for i in range(self.degrees_of_freedom):
            temp_angle = JointAngle()
            temp_angle.joint_identifier = i
            temp_angle.value = 0.0
            req.input.joint_angles.joint_angles.append(temp_angle)

        # Send the angles
        rospy.loginfo("Sending the robot vertical...")
        try:
            self.play_joint_trajectory(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call PlayJointTrajectory")
            return False
        else:
            return self.wait_for_action_end_or_abort()

    def example_send_gripper_command(self, value):
        # Initialize the request
        # Close the gripper
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION

        rospy.loginfo("Sending the gripper command...")

        # Call the service
        try:
            self.send_gripper_command(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            return False
        else:
            time.sleep(0.5)
            return True

    gripper_cur_pos = 0.0
    def read_gripper_pos(self, data):
        #interconnect:oneof_tool_feedback:gripper_feedback:motor:position
        #rospy.loginfo(data.interconnect.oneof_tool_feedback.gripper_feedback[0].motor.position)
        self.gripper_cur_pos = data.interconnect.oneof_tool_feedback.gripper_feedback[0].motor[0].position/100
        #rospy.loginfo("Gripper pos is: "+str(self.gripper_cur_pos))

    def go_to_gripper_pose(self, command):

        #subscribe to where the current position of the finger is
        #sub = rospy.Subscriber("/"+self.robot_name+"/gen3_lite_2f_gripper_controller/gripper_cmd/feedback", InterconnectCyclic_Feedback, self.read_gripper_pos)
        #rostopic echo /my_gen3_lite/base_feedback
        sub = rospy.Subscriber("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback, self.read_gripper_pos)

        req = SendGripperCommandRequest()
        finger = Finger()

        finger1 = command["gripper"]["finger"][0]

        finger.finger_identifier = 1#finger1["fingerIdentifier"] #<< this is wrong for some reason
        finger.value = min(max(0.01,finger1["value"]),0.99)
        rospy.loginfo(finger.value)

        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION
        # Call the service
        try:
            self.send_gripper_command(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            sub.unregister() #unsubscribe the gripper position since we don't need it anymore
            return False
        else:
            #wait until the gripper is within a margin of error of the wanted position
            while abs(finger.value - self.gripper_cur_pos) > 0.03:
                time.sleep(0.5)
            sub.unregister()
            return True
            #return self.wait_for_action_end_or_abort()


    def execute_sequence(self, seq_name):
        seqIndex = -1
        i = 0
        # find the sequence in the sequence list
        for sequence in rospy.get_param("/sequences/sequence"):
            #rospy.loginfo(sequence["name"])
            if sequence["name"] == seq_name:
                seqIndex = i
                rospy.loginfo(sequence["name"])
                break
            i += 1

        if seqIndex == -1:
            rospy.loginfo("Could not find the sequence \""+seq_name+"\"")
            # fail
            return False

        # now for each task in that sequence, execute the corresponding action
        tasks = rospy.get_param("/sequences/sequence")[seqIndex]["tasks"]
        success = True
        for task in tasks:
            if "sendGripperCommand" in task["action"]:
                rospy.loginfo("Execute Gripper action")
                success &= self.go_to_gripper_pose(task["action"]["sendGripperCommand"])
                #success &= self.example_send_gripper_command(0.5)#task["action"]["sendGripperCommand"]["gripper"]["finger"][0]["value"])
            elif "reachPose" in task["action"]:
                rospy.loginfo("Execute ReachPose action")
                success &= self.go_to_cart_pose(task["action"]["reachPose"])
            elif "reachJointAngles" in task["action"]:
                rospy.loginfo("Execute Joint Angle Action")
                #success &= go_gripper_pose()
            else:
                success = False
                rospy.loginfo("Invalid Command Type")

        return success

    def go_param_pose(self, index):
        reachPose = rospy.get_param("/positions/poses/pose")[index]["reachPose"]
        return go_to_cart_pose(reachPose)

    def go_to_cart_pose(self, reachPose):
        tPose = reachPose["targetPose"]
        req = PlayCartesianTrajectoryRequest()
        req.input.target_pose.x = tPose["x"]
        req.input.target_pose.y = tPose["y"]
        req.input.target_pose.z = tPose["z"]
        req.input.target_pose.theta_x = tPose["thetaX"]
        req.input.target_pose.theta_y = tPose["thetaY"]
        req.input.target_pose.theta_z = tPose["thetaZ"]

        # Call the service
        rospy.loginfo("Sending the robot to the cartesian pose...")
        try:
            self.play_cartesian_trajectory(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call PlayCartesianTrajectory")
            return False
        else:
            return self.wait_for_action_start_n_finish()#self.wait_for_action_end_or_abort()


    def preplanned_ex(self):
        return go_param_pose(0)

    grabbed = True
    def grab_toggle(self):
        if self.grabbed:
            self.execute_sequence("put_back_cart")
            self.grabbed = False
        else:
            self.execute_sequence("grab_cart")
            self.grabbed = True


    """  
        #constr = reachPose["constraint"]
        #pose_speed = CartesianSpeed()
        #pose_speed.translation = constr["speed"]["translation"]
        #pose_speed.orientation = constr["speed"]["orientation"]

        # The constraint is a one_of in Protobuf. The one_of concept does not exist in ROS
        # To specify a one_of, create it and put it in the appropriate list of the oneof_type member of the ROS object :
        #req.input.constraint.oneof_type.speed.append(pose_speed)"""


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

            # PS axes are same, some button diffs
            # 2==Triangle, 3==square
            # stick btns are 11 (L) and 12 (R)
            # 8 9 10 =>share, options, ps btn
            # 6,7 are triggers all the way down
        # Future improvement: instead just track what the last button was and its count rather than an array
        # this also means you can't get a double press by rocking back btw 2 btns
        # or just have a mechanism to cancel if we switch btns, but for now this is good enough
    button_ups = [0, 0, 0, 0]
    lastButtonDown = [False, False, False, False]
    buttonDown = [False, False, False, False]
    onButtonUp = [False, False, False, False]

    def double_pressed(self, btnIndex, event):
        if self.button_ups[btnIndex] > 1:
            rospy.loginfo("Double Pressed")
            rospy.loginfo(btnIndex)
            rospy.loginfo("##############")
            if btnIndex == 1:
                self.execute_sequence("Poke_cart")
        else:
            rospy.loginfo("Single Pressed")
            rospy.loginfo(btnIndex)
            rospy.loginfo("##############")
            # single press B for a gripper toggle
            if btnIndex == 1:
                self.grab_toggle()
                #self.toggle_gripper()
            #if btnIndex ==
        self.button_ups[btnIndex] = 0

    # export ROS_MASTER_URI=http://localhost:11311, unset ROS_IP for each terminal
    # launch each of these in their own terminal
    # roscore
    # roslaunch kortex_gazebo spawn_kortex_robot.launch arm:=gen3_lite
    # rosparam set joy_node/dev "/dev/input/jsX(2), rosrun joy joy_node
    # roslaunch pk_control control.launch

    def check_press(self, btnIndex, btnState):
        # rospy.loginfo("Do Check Press")
        self.lastButtonDown[btnIndex] = self.buttonDown[btnIndex]
        if btnState == 1:
            self.buttonDown[btnIndex] = True
        else:
            self.buttonDown[btnIndex] = False

        self.onButtonUp[btnIndex] = self.lastButtonDown[btnIndex] and not self.buttonDown[btnIndex]

        if self.onButtonUp[btnIndex]:
            if self.button_ups[btnIndex] == 0:
                pressedCallback = partial(self.double_pressed,
                                          btnIndex)  # make a callback "partial" func which has btnIndex filled out
                rospy.Timer(rospy.Duration(0.2), pressedCallback, oneshot=True)
            self.button_ups[btnIndex] += 1
            rospy.loginfo(self.button_ups[btnIndex])

    def handle_controller(self, data):
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
            #rospy.loginfo("Escaped")
            rospy.signal_shutdown("User Exited")
            #self.escaped = True

    def main(self):
        # For testing purposes
        success = self.is_init_success
        try:
            rospy.delete_param("/kortex_examples_test_results/full_arm_movement_python")
        except:
            pass

        if success:
            # *******************************************************************************
            # Make sure to clear the robot's faults else it won't move if it's already in fault
            success &= self.example_clear_faults()
            # *******************************************************************************
            #rospy.loginfo("hello?")
            # *******************************************************************************
            # Activate the action notifications
            success &= self.example_subscribe_to_a_robot_notification()
            # *******************************************************************************

            # *******************************************************************************
            # Move the robot to the Home position with an Action
            #success &= self.example_home_the_robot()
            # *******************************************************************************

            # *******************************************************************************
            # Example of gripper command

            # Set the reference frame to "Mixed"
            success &= self.example_set_cartesian_reference_frame()



            success &= self.example_send_gripper_command(0.0)
            self.gripper_closed = False

            if rospy.has_param("/use_joy"):#"/joy_node/dev"):#example.is_controller_present:
                rospy.loginfo("Using Joystick Input")
                rospy.Subscriber("/joy", Joy, self.handle_controller)
                rospy.spin()
            else:
                rospy.loginfo("Doing Demo Sequence")

                success &= self.example_send_gripper_command(0.0)

                rospy.loginfo("do Pre-planned example")
                #self.preplanned_ex()
                self.execute_sequence("grab_cart")

            # *******************************************************************************

            # *******************************************************************************
            # Set the reference frame to "Mixed"
            # success &= self.example_set_cartesian_reference_frame()

            # Example of cartesian pose
            # Let's make it move in Z
            # success &= self.example_send_cartesian_pose()
            # *******************************************************************************

            # *******************************************************************************
            # Example of angular position
            # Let's send the arm to vertical position
            # success &= self.example_send_joint_angles()
            # *******************************************************************************

            # *******************************************************************************
            # Example of gripper command
            # Let's close the gripper at 50%
            # if self.is_gripper_present:
            #    success &= self.example_send_gripper_command(0.5)
            # else:
            #    rospy.logwarn("No gripper is present on the arm.")
            # *******************************************************************************

        # For testing purposes
        # rospy.set_param("/kortex_examples_test_results/full_arm_movement_python", success)

        if not success:
            rospy.logerr("The example encountered an error.")

if __name__ == "__main__":
    ex = Control()
    ex.main()


    """def handle_controller(self, data):
        # pprint(vars(data))
        # rospy.loginfo(data)
        btnA = data.buttons[0]
        btnB = data.buttons[1]
        ax0 = data.axes[0]

        if btnA == 1:
            rospy.loginfo("Escaped")
            self.escaped = True

        if btnB == 1:
            if self.gripper_closed:
                self.example_send_gripper_command(1.0)
                self.gripper_closed = False
                rospy.loginfo("open")
            else:
                self.example_send_gripper_command(0.0)
                self.gripper_closed = True
                rospy.loginfo("close")"""

        # rospy.loginfo("btnA is: " + str(btnA) + " ax0 is" + str(ax0))

"""
from rostopic echo /my_gen3_lite/gen3_lite_2f_gripper_controller/gripper_cmd/status

header: 
  seq: 11502
  stamp: 
    secs: 2288
    nsecs: 400000000
  frame_id: ''
status_list: 
  - 
    goal_id: 
      stamp: 
        secs: 510
        nsecs: 404000000
      id: "/my_gen3_lite/my_gen3_lite_driver-65-510.404000000"
    status: 3
    text: ''



from rostopic echo /my_gen3_lite/gen3_lite_2f_gripper_controller/gripper_cmd/feedback

actuators:
    ...
interconnect:
  oneof_tool_feedback: 
    gripper_feedback: 
      - 
        feedback_id: 
          identifier: 0
        status_flags: 0
        fault_bank_a: 0
        fault_bank_b: 0
        warning_bank_a: 0
        warning_bank_b: 0
        motor: 
          - 
            motor_id: 0
            position: 0.998776376247406
            velocity: 0.0
            current_motor: 0.0
            voltage: 0.0
            temperature_motor: 0.0
---

"""


