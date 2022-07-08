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

import numpy as np # for vector math, might be able to replace this with a ros version later
import math

# debug
# from pprint import pprint




# rotate a vector about an axis theta radians, taken from https://stackoverflow.com/questions/6802577/rotation-of-3d-vector
# which implements the euler-rodrigues formula
# this returns a rotation matrix you dot multiply your vector by
def rotation_matrix(axis, theta):
    """
    Return the rotation matrix associated with counterclockwise rotation about
    the given axis by theta radians.
    """
    axis = np.asarray(axis)
    axis = axis / math.sqrt(np.dot(axis, axis))
    a = math.cos(theta / 2.0)
    b, c, d = -axis * math.sin(theta / 2.0)
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                     [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                     [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])

#returns the rotated vector, vec and axis are both [x,y,z], theta is radians
def rotate_vector(vec, axis, theta):
    rot_matrix = rotation_matrix(axis,theta)
    return np.dot(rot_matrix,vec)

# generate the points of an n-gon (use this for generating circular motion)
def generate_circle(radius=1, num_pts=8):
    pts = []
    tau = 6.283185 # tau= 2*pi ~=6.283185
    angle = tau/num_pts
    for i in range(num_pts):
        pt_angle = angle*i
        x = math.cos(pt_angle)*radius
        y = math.sin(pt_angle)*radius
        pts.append([x,y, pt_angle])
    pts.append(pts[0]) #add the starting point to the end for a complete circle
    return pts

#generate a circle around the x (forward) axis in 3d
def gen_circle_axis(radius=0.5, num_pts=8, origin=[0.3812,0.0644,0.2299]):
    circle = generate_circle(radius, num_pts)
    rospy.loginfo(circle)
    res = []
    for pt in circle:
        #if pt[2]
        res.append([origin[0],origin[1]+pt[0],origin[2]+pt[1], pt[2]*57.2958])
    res.append([origin[0],origin[1],origin[2],0])
    return res

#return the distance between 2 vector3s (unlike above, they have (x,y,z) (tuples?) rather than [x,y,z]
def vec_dist(a,b):
    return np.linalg.norm(a - b)

# take a vec3 position, limit it to something in reach of the arm
# judging by full_forward, base height (origin) is around 0.25 or 0.26 (z axis >> up)
# and the max distance is 0.75 or 0.76
def spherical_dist_bound(vec, max_dist=0.74, origin=[0,0,0.25]):
    np_origin = np.array(origin)
    dist = vec_dist(vec,np_origin)#vec_dist(np_origin,vec)
    rospy.loginfo("vec")
    rospy.loginfo(vec)
    rospy.loginfo("dist")
    rospy.loginfo(dist)
    #rospy.loginfo(max)

    ret = vec
    if dist > max_dist:
        ratio = max_dist/dist
        temp = (vec-np_origin)*ratio
        ret = temp + np_origin
        rospy.loginfo("limit")
        rospy.loginfo(ratio)
    if ret[2] > 0.45: #limit the height
        ret[2] = 0.45
    return ret

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
            # see https://github.com/Kinovarobotics/ros_kortex/tree/noetic-devel/kortex_driver/srv/generated/base
            # which has all the different message in types
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

            # more actions

            # stop smoothly
            # rostopic pub /my_gen3/in/stop std_msgs/Empty "{}"
            send_stop_command_full_name = '/' + self.robot_name + '/base/stop'
            rospy.wait_for_service(send_stop_command_full_name)
            self.send_stop_command = rospy.ServiceProxy(send_stop_command_full_name, Stop)

            rospy.loginfo("J")
            #emergency stop
            # rostopic pub /my_gen3/in/emergency_stop std_msgs/Empty "{}"

            #clear faults
            # rostopic pub /my_gen3/in/clear_faults std_msgs/Empty "{}" << already have it

            #cart vel << already have it?
            # rostopic pub /my_gen3/in/cartesian_velocity kortex_driver/TwistCommand "reference_frame: 0
            # twist: {linear_x: 0.0, linear_y: 0.0, linear_z: 0.05, angular_x: 0.0, angular_y: 0.0,
            # angular_z: 0.0}
            # duration: 0"

            # joint vel << have it?
            # rostopic pub /my_gen3/in/joint_velocity kortex_driver/Base_JointSpeeds "joint_speeds:
            # - joint_identifier: 0
            #   value: -0.57
            #   duration: 0"

            joint_velocity_full_name = '/' + self.robot_name + '/base/send_selected_joint_speed_command'
            rospy.wait_for_service(joint_velocity_full_name)
            self.joint_velocity_command = rospy.ServiceProxy(joint_velocity_full_name, SendSelectedJointSpeedCommand)#JointSpeed)


        except:
            self.is_init_success = False
        else:
            self.is_init_success = True

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event
        #rospy.loginfo("Last action was "+str(notif.action_event))
        #if (self.last_action_notif_type == ActionEvent.ACTION_START):
        #    rospy.loginfo("Received ACTION_START notification")
        if (self.last_action_notif_type == ActionEvent.ACTION_END):
            rospy.loginfo("Received ACTION_END notification")
        """if (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
            rospy.loginfo("Received ACTION_ABORT notification")
        if (self.last_action_notif_type == ActionEvent.ACTION_FEEDBACK):
            rospy.loginfo("Received ACTION_FEEDBACK notification")
        if (self.last_action_notif_type == ActionEvent.ACTION_PAUSE):
            rospy.loginfo("Received ACTION_PAUSE notification")
        if (self.last_action_notif_type == ActionEvent.ACTION_PREPROCESS_ABORT):
            rospy.loginfo("Received ACTION_PREPROCESS_ABORT notification")
        if (self.last_action_notif_type == ActionEvent.ACTION_PREPROCESS_END):
            rospy.loginfo("Received ACTION_PREPROCESS_END notification")
        if (self.last_action_notif_type == ActionEvent.ACTION_PREPROCESS_START):
            rospy.loginfo("Received ACTION_PREPROCESS_START notification")"""

    def wait_for_action_start_n_finish(self):
        while not rospy.is_shutdown():
            if(self.last_action_notif_type == ActionEvent.ACTION_START):
                #rospy.loginfo("Received ACTION_START notification")
                return self.wait_for_action_end_or_abort()

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                #rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                #rospy.loginfo("Received ACTION_ABORT notification")
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

    def control_angle_vel(self):
        #/ my_gen3_lite / actuator_config / set_control_mode

        x = SendSelectedJointSpeedCommand()#JointSpeed()#
        x.input = JointSpeed()

        x.joint_identifier = 4
        x.value = 10
        x.duration = 00
        x.input.joint_identifier = 4
        x.input.value = 10
        x.input.duration = 00

        rospy.loginfo("Send Angle Velocity")

        self.joint_velocity_command(x)

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
            return self.wait_for_action_start_n_finish()#self.wait_for_action_end_or_abort()

    def FillCartesianWaypoint(self, new_x, new_y, new_z, new_theta_x, new_theta_y, new_theta_z, blending_radius):
        waypoint = Waypoint()
        cartesianWaypoint = CartesianWaypoint()

        cartesianWaypoint.pose.x = new_x
        cartesianWaypoint.pose.y = new_y
        cartesianWaypoint.pose.z = new_z
        cartesianWaypoint.pose.theta_x = new_theta_x
        cartesianWaypoint.pose.theta_y = new_theta_y
        cartesianWaypoint.pose.theta_z = new_theta_z
        cartesianWaypoint.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_BASE
        cartesianWaypoint.blending_radius = blending_radius
        waypoint.oneof_type_of_waypoint.cartesian_waypoint.append(cartesianWaypoint)

        return waypoint

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
        #lite_2f_gripper_controller feedback wasn't giving anything in simulation
        #sub = rospy.Subscriber("/"+self.robot_name+"/gen3_lite_2f_gripper_controller/gripper_cmd/feedback", InterconnectCyclic_Feedback, self.read_gripper_pos)

        #but rostopic echo /my_gen3_lite/base_feedback did
        sub = rospy.Subscriber("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback, self.read_gripper_pos)

        req = SendGripperCommandRequest()
        finger = Finger()

        finger1 = command["gripper"]["finger"][0]

        finger.finger_identifier = 1#finger1["fingerIdentifier"] #for now, hardcoded
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


    prevReach = { #default previous position
        "targetPose":{
            "x": 0.38117900490760803,
            "y": 0.0644112303853035,
            "z": 0.22991076111793518
        }
    }
    seq_cancelled = False
    def execute_sequence(self, seq_name):
        self.seq_cancelled = False
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
        return self.sequence_for_loop(tasks)

    def sequence_for_loop(self, tasks):
        success = True
        self.prevReach = tasks[0]["action"]["reachPose"]
        for task in tasks[1:]: #skip the first task, its the relative position's origin
            if self.seq_cancelled:
                self.seq_cancelled = False
                break
            if "sendGripperCommand" in task["action"]:
                rospy.loginfo("Execute Gripper action")
                success &= self.go_to_gripper_pose(task["action"]["sendGripperCommand"])
                #success &= self.example_send_gripper_command(0.5)#task["action"]["sendGripperCommand"]["gripper"]["finger"][0]["value"])
            elif "reachPose" in task["action"]:
                rospy.loginfo("Execute ReachPose action")
                # success &= self.go_to_cart_pose(task["action"]["reachPose"])
                success &= self.go_to_cart_pose_relative(task["action"]["reachPose"], self.prevReach)
                self.prevReach = task["action"]["reachPose"]
            elif "reachJointAngles" in task["action"]:
                rospy.loginfo("Execute Joint Angle Action")
                #success &= go_gripper_pose()
            else:
                success = False
                rospy.loginfo("Invalid Command Type")

        return success

    def go_param_pose(self, index):
        reachPose = rospy.get_param("/positions/poses/pose")[index]["reachPose"]
        return self.go_to_cart_pose(reachPose)

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

    def go_to_cart_pose_relative(self, reachPose, prevReachPose):
        feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)

        tPose = reachPose["targetPose"]
        prevTPose = prevReachPose["targetPose"]
        #need to do newPose-prevPose (both from the seq file) to get relative movement
        #then add this relative position to the previously recorded absolute position (feedback_pos)

        vec = [
            tPose["x"],
            tPose["y"],
            tPose["z"]
        ]
        prev_vec = [
            prevTPose["x"],
            prevTPose["y"],
            prevTPose["z"]
        ]
        feedback_vec = [
            feedback.base.commanded_tool_pose_x,
            feedback.base.commanded_tool_pose_y,
            feedback.base.commanded_tool_pose_z
        ]
        nparr_vec = np.array(vec)
        nparr_prev_vec = np.array(prev_vec)
        nparr_feedback = np.array(feedback_vec)

        added_vec = (nparr_vec - nparr_prev_vec) #make it a relative movement (default relative to retracted)
        added_vec = added_vec + nparr_feedback #add the rel to our actual previous position

        #spherical dist bound subtracts the ordinary origin, then we can add our starting point
        vec_res = spherical_dist_bound(added_vec,0.73)

        req = PlayCartesianTrajectoryRequest()
        req.input.target_pose.x = vec_res[0]
        req.input.target_pose.y = vec_res[1]
        req.input.target_pose.z = vec_res[2]
        req.input.target_pose.theta_x = tPose["thetaX"]#feedback.base.commanded_tool_pose_theta_x + tPose["thetaX"]
        req.input.target_pose.theta_y = tPose["thetaY"]#feedback.base.commanded_tool_pose_theta_y + tPose["thetaY"]
        req.input.target_pose.theta_z = tPose["thetaZ"]#feedback.base.commanded_tool_pose_theta_z + tPose["thetaZ"]



        # Call the service
        rospy.loginfo("Sending the robot to the relative cartesian pose...")
        rospy.loginfo(tPose)
        rospy.loginfo(req.input.target_pose)
        try:
            self.play_cartesian_trajectory(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call PlayCartesianTrajectory")
            return False
        else:
            return self.wait_for_action_start_n_finish()


    def stop_seq(self):
        self.seq_cancelled = True
        self.send_stop_command()

    #def preplanned_ex(self):
    #    return go_param_pose(0)

    #TODO Verify:
    grabbed = False
    def grab_toggle(self):
        msg = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)
        self.read_gripper_pos(msg)
        #rospy.loginfo(msg)
        #rospy.loginfo("===========")
        #if self.grabbed:
        if self.gripper_cur_pos > 0.95:#< 0.05: #closed
            self.execute_sequence("put_back_cart")
            self.grabbed = False
        else:
            self.execute_sequence("grab_cart")
            self.grabbed = True


    #generate and execute a circular motion given a radius
    #wrist_fixed determines whether the gripper stays at the same angle or whether
    #the angle changes relative to where on the circle the arm is
    #wrist angle chooses the angle to put the gripper at
    #TODO: it sounds like the Cartesian trajectory action server can do this, but smoothly
    def do_circle(self, radius=0.20, wrist_fixed=False, wrist_angle=90):
        pts = gen_circle_axis(radius,12)
        rospy.loginfo("___________ RES ___________")
        rospy.loginfo(pts)
        #tasks[0]["action"]["reachPose"] << what it should look like
        tasks = [
            { #default previous position
                "action" :{
                    "reachPose":{
                        "targetPose": {
                            "x": 0.38117900490760803,
                            "y": 0.0644112303853035,
                            "z": 0.22991076111793518
                        }
                    }
                }
            }
        ]
        for pt in pts:
            tasks.append({"action":{"reachPose":{"targetPose":{
                "x":pt[0],
                "y":pt[1],
                "z":pt[2],
                #"thetaX": 92.8780517578125,
                #"thetaY": -3.9708383083343506,
                #"thetaZ": 92.5357894897461
                "thetaX": 90,
                "thetaY": 0,#180-pt[3],
                "thetaZ": 90
            }}}})
        rospy.loginfo("***** TASKS *****")
        rospy.loginfo(tasks)
        self.sequence_for_loop(tasks)

        """    prevReach = { #default previous position
        "targetPose":{
            "x": 0.38117900490760803,
            "y": 0.0644112303853035,
            "z": 0.22991076111793518
        }
        }"""

    #TODO: Temporary function to help test relative movement
    #in the future, up and down movement itself needs to be relative/additive to other
    #motions (so we can swing up/down)
    #or should it?
    #I think retracted_cart should be the origin
    up = False
    def up_down_toggle(self):
        if self.up:
            self.up = False
            rospy.loginfo("Go Down")
            return self.go_param_pose(0)
        else:
            self.up = True
            rospy.loginfo("Go Up")
            return self.go_param_pose(1)

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
            self.button_ups[btnIndex] = 0
            #rospy.loginfo("Double Pressed")
            #rospy.loginfo(btnIndex)
            #rospy.loginfo("##############")
            if btnIndex == 0:
                self.control_angle_vel() #test twisting
            if btnIndex == 1:
                self.execute_sequence("Poke_cart")
                grabbed = False
            if btnIndex == 3:
                rospy.loginfo("Go Down")
                self.execute_sequence("down")
            #if btnIndex == 9: #reset to home
            #    self.example_home_the_robot()
            #    self.go_param_pose(0)
        else:
            self.button_ups[btnIndex] = 0
            #rospy.loginfo("Single Pressed")
            #rospy.loginfo(btnIndex)
            #rospy.loginfo("##############")
            # single press B for a gripper toggle
            if btnIndex == 0:
                rospy.loginfo("do circle")
                self.do_circle()
            if btnIndex == 1:
                rospy.loginfo("do grab toggle")
                self.grab_toggle()
            if btnIndex == 2:
                 #cancel the movement
                 rospy.loginfo("Stop sequence")
                 self.stop_seq()
            if btnIndex == 3:
                rospy.loginfo("Go Up")
                self.execute_sequence("up")
                # self.up_down_toggle()
            #if btnIndex == 9:
            #    self.go_param_pose(0)


            #if btnIndex ==


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
                # make a callback "partial" func which has btnIndex filled out
                pressedCallback = partial(self.double_pressed, btnIndex)
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
        btnX = data.buttons[2]
        btnY = data.buttons[3]

        btnEscape = data.buttons[8] #share on ps, xbox btn on xbox
        btnOptions = data.buttons[9] #stbl on xbox
        ax0 = data.axes[0]

        self.check_press(0, btnA)
        self.check_press(1, btnB)
        self.check_press(2, btnX)
        self.check_press(3, btnY)
        #self.check_press(9, btnOptions)

        if btnEscape == 1:
            #rospy.loginfo("Escaped")
            rospy.signal_shutdown("User Exited")
            #self.escaped = True

        #go to home, reset
        if btnOptions == 1:
            self.example_home_the_robot()
            self.go_param_pose(0)

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
            success &= self.example_home_the_robot()
            self.go_param_pose(0)
            # *******************************************************************************

            # *******************************************************************************
            # Example of gripper command

            # Set the reference frame to "Mixed"
            success &= self.example_set_cartesian_reference_frame()



            success &= self.example_send_gripper_command(0.0)
            self.gripper_closed = False

            # rospy.has_param("/use_joy") and
            if rospy.get_param("/use_joy")==True:#"/joy_node/dev"):#example.is_controller_present:
                rospy.loginfo("Using Joystick Input")
                rospy.Subscriber("/joy", Joy, self.handle_controller)
                rospy.spin()
            else:
                rospy.loginfo("Doing Demo Sequence")

                #rospy.loginfo("Do Built-in stuff")

                #success &= self.example_send_gripper_command(0.5)
                #success &= self.example_send_cartesian_pose()

                #rospy.loginfo("do Pre-planned example")
                #self.execute_sequence("grab_cart")
                #self.up_down_toggle()
                #rospy.sleep(0.25)
                #self.execute_sequence("grab_cart")
                #self.up_down_toggle()

                self.execute_sequence("Circle")

                #self.go_to_cart_pose_relative()
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


