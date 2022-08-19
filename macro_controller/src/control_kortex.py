#!/usr/bin/env python3

import rospy
import time
from kortex_driver.srv import *
from kortex_driver.msg import *
from axis_camera.msg import Axis

# import controller for example like https://andrewdai.co/xbox-controller-ros.html#rosjoy
from sensor_msgs.msg import Joy
from functools import partial #used to pass arguments in a callback

#np isn't installed on Jackal, so it can't be used, use math instead
import math

# generate the points of an n-gon given n and radius, etc (use this for generating circular motion)
def generate_circle(radius=1, num_pts=8, dir=1, offset=0):
    pts = []
    tau = 6.283185 # tau= 2*pi ~=6.283185
    angle = tau/num_pts
    for i in range(num_pts):
        pt_angle = (dir*angle*i)+offset #offset needs to be in radians
        x = math.cos(pt_angle)*radius
        y = math.sin(pt_angle)*radius
        pts.append([x,y, pt_angle])
    pts.append(pts[0]) #add the starting point to the end for a complete circle
    return pts

#generate a circle around the x (forward) axis in 3d, centered at an origin
def gen_circle_axis(radius=0.5, num_pts=8, origin=[0.40,0.0644,0.2299], dir=1, offset=0):
    rospy.loginfo("Offset is:")
    rospy.loginfo(offset)
    rospy.loginfo(offset/57.2958)
    circle = generate_circle(radius, num_pts, dir, offset/57.2958)
    rospy.loginfo(circle)
    res = []
    for pt in circle:
        #if pt[2]
        res.append([origin[0],origin[1]+pt[0],origin[2]+pt[1], pt[2]*57.2958])
    res.append([origin[0],origin[1],origin[2],0])
    return res

"""Some vector manipulation functions since we don't have numpy"""
#return the distance between 2 vector3s (unlike above, they have (x,y,z) (tuples?) rather than [x,y,z]
def vec_dist(a,b):
    vec = vec_sub(a,b)
    sum = 0
    for i in vec:
        sum += i*i
    return math.sqrt(sum)

def vec_sub(a,b):
    result = []
    for val1, val2 in zip(a, b):
        result.append(val1 - val2)
    return result

def vec_add(a,b):
    result = []
    for val1, val2 in zip(a, b):
        result.append(val1 + val2)
    return result

def vec_dot_mul(a,b):
    result = []
    for val1, val2 in zip(a, b):
        result.append(val1 * val2)
    return result

#perhaps these math utilities should be moved to their own file, and a new vector type made
# complete with operator overloading https://www.programiz.com/python-programming/operator-overloading

# rotate a vector about an axis theta radians, taken from https://stackoverflow.com/questions/6802577/rotation-of-3d-vector
# which implements the euler-rodrigues formula
# this returns a rotation matrix you dot multiply your vector by
"""
Return the rotation matrix associated with counterclockwise rotation about
the given axis by theta radians.
** this is untested
"""
def rotation_matrix(axis, theta):
    axis = axis / math.sqrt(vec_dot_mul(axis, axis))
    a = math.cos(theta / 2.0)
    b, c, d = -axis * math.sin(theta / 2.0)
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    return [
        [aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
        [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
        [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]
    ]

# returns the rotated vector, vec and axis are both [x,y,z], theta is radians
def rotate_vector(vec, axis, theta):
    rot_matrix = rotation_matrix(axis, theta)
    return vec_dot_mul(rot_matrix, vec)

# Function to limit the position of the arm to areas where it (probably) won't get stuck
# take a vec3 position, limit it to something in reach of the arm
# judging by full_forward position, the base height (origin) is around 0.25 or 0.26
# and the max distance is 0.75 or 0.76
# ** Note that for the arm: z=up, y=side to side, x=Forwards, in the vector3 x=axis 0, y=1, z=2
def spherical_dist_bound(vec, max_dist=0.74, origin=[0,0,0.25]):
    np_origin = origin
    dist = vec_dist(vec, np_origin)
    #rospy.loginfo("vec")
    #rospy.loginfo(vec)
    #rospy.loginfo("dist")
    #rospy.loginfo(dist)

    ret = vec
    if dist > max_dist:
        ratio = max_dist/dist
        temp = vec_sub(vec, np_origin)
        ret = vec_add(temp, np_origin)
        rospy.loginfo("limit")
        rospy.loginfo(ratio)
    if ret[2] > 0.45: #limit the height, unneeded due to ret=min_dist_bound
        ret[2] = 0.45
    ret = min_dist_bound(ret)
    return ret

#keep a min distance away from the arm base, max + min heights
def min_dist_bound(vec):#, base, max_h, min_h):
    #1down =                                 = 0.11
    #base  = 0.22991076111793518            ~= 0.23
    #1up   = 0.2299+0.35-0.22991076111793518 = 0.35
    #2up   = 0.12 + 0.35                     = 0.47
    #retracted (x) = 0.38117 ~= 0.38
    #extended  (x) = 0.75934 ~= 0.76

    #dist from base bound
    vec_out = vec
    if(vec[0] < 0.38):
        vec_out[0] = 0.38
    if(vec[0] > 0.76):
        vec_out[0] = 0.76
    #max/min vertical
    if(vec[2] > 0.47):
        vec_out[2] = 0.47
    elif(vec[2] < 0.11):
        vec_out[2] = 0.11
    return vec_out

#for most actions, we won't want L/R deviation in order for them to work, so selectively
# apply this function
def horizontal_dist_bound(vec):
    vec_out = vec
    if(vec[1] > 0.0645):
        vec_out[1] = 0.06448
    elif(vec[1] < 0.064):
        vec_out[1] = 0.06448
    return vec_out

# derived from the ExampleFullArmMove python class in the ros_kortex examples
class Control:
    #did we cancel an action?
    escaped = False
    #was the gripper last reported as open or closed?
    gripper_closed = False

    def __init__(self):
        try:
            #Get various parameters we will use in the program from rosparams
            #Then subscribe to various rostopics to get information from the robot
            #and make functions that correspond to various rosservices to send messages to
            # the arm

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

            play_selected_joint_trajectory_full_name = '/' + self.robot_name + '/base/play_selected_joint_trajectory'
            rospy.wait_for_service(play_selected_joint_trajectory_full_name)
            self.play_selected_joint_trajectory = rospy.ServiceProxy(play_selected_joint_trajectory_full_name, PlaySelectedJointTrajectory)

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

            rospy.loginfo("J") #set the axis camera to forward
            self.cmd = rospy.Publisher("/axis/cmd", Axis, queue_size=1)

        except:
            self.is_init_success = False
        else:
            self.is_init_success = True

    # a callback function which will update the last notification we received from the robot
    # of interest are the ACTION_START, ACTION_END, and ACTION_ABORT, which tell us if
    # a ros_kortex sequence of arm movements started, or ended
    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event
        if (self.last_action_notif_type == ActionEvent.ACTION_END):
            rospy.loginfo("Received ACTION_END notification")

    # This functions checks that we recently received for a start and an end of a sequence
    # (used in conjunction with a loop to wait for a sequence to complete)
    # This was made to fix a problem where sequences would see the previous sequence's "ACTION_END"
    # and thus never execute
    def wait_for_action_start_n_finish(self):
        while not rospy.is_shutdown():
            if(self.last_action_notif_type == ActionEvent.ACTION_START):
                return self.wait_for_action_end_or_abort()

    # checks for the ACTION_END or ACTION_ABORT event
    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                return False
            else:
                time.sleep(0.01)

    # from the example, activates publishing of action notifications
    def example_subscribe_to_a_robot_notification(self):
        # Activate the publishing of the ActionNotification
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

    #Clears any faults (halting errors) from the kinova arm so we can use it
    # A future improvement would be to find a way to detect a fault, and
    # when one occurs, wait for an input to send it to a safe position
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

    # Sends the robot to a home position (kind-of imagine holding your forearm in front of your
    # face)
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
                return self.wait_for_action_start_n_finish()

    # There are 3 reference frames to use, essentially global, or local coordinates,
    # or a mixture of global position, but local rotation coordinates
    # (see https://github.com/Kinovarobotics/kortex/blob/master/api_python/doc/markdown/enums/Common/CartesianReferenceFrame.md)
    # This function sets the arm to use the mixed reference frame
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

    #
    def rotate_wrist(self, angle, abs_angle=False):
        self.last_action_notif_type = None
        # Get the actual cartesian pose to increment it
        # You can create a subscriber to listen to the base_feedback
        # Here we only need the latest message in the topic though
        feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)

        # Call the service

        req = PlayJointTrajectoryRequest()
        for i in range(self.degrees_of_freedom):
            temp_angle = JointAngle()
            temp_angle.joint_identifier = i
            temp_angle.value = feedback.actuators[i].position
            if i == self.degrees_of_freedom-1:
                if abs_angle:
                    temp_angle.value = angle
                else:
                    temp_angle.value += angle
            req.input.joint_angles.joint_angles.append(temp_angle)

        rospy.loginfo("Sending the robot to the cartesian pose...")
        try:
            self.play_joint_trajectory(req)
            feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)
            rospy.loginfo(feedback.actuators[5])
        except rospy.ServiceException:
            rospy.logerr("Failed to call PlayCartesianTrajectory")
            return False
        else:
            return self.wait_for_action_start_n_finish()#self.wait_for_action_end_or_abort()

    # Example for moving in a sequence using the built-in cartesian waypoint list
    # For now there is no advantage to using it over sequence_for_loop,
    # as it doesn't work in simulation, while accomplishing the same thing as sequence_for_loop
    # However, it doesn't use deprecated functions like sequence_for_loop, so it may become
    # necessary to use this method in the future
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

    #Example for sending the gripper commands
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
        rospy.loginfo(req)
        # Call the service
        try:
            self.send_gripper_command(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            return False
        else:
            time.sleep(0.5)
            return True

    #Function to read the current gripper position, since one isn't provided by default
    # by ros_kortex
    gripper_cur_pos = 0.0
    def read_gripper_pos(self, data):
        #interconnect:oneof_tool_feedback:gripper_feedback:motor:position
        #rospy.loginfo(data.interconnect.oneof_tool_feedback.gripper_feedback[0].motor.position)
        self.gripper_cur_pos = data.interconnect.oneof_tool_feedback.gripper_feedback[0].motor[0].position/100
        #rospy.loginfo("Gripper pos is: "+str(self.gripper_cur_pos))

    #Conntinually opens/closes the gripper until the gripper is within a threshold of a
    # specified position
    def go_to_gripper_pose(self, command):
        #subscribe to where the current position of the finger is
        #lite_2f_gripper_controller feedback wasn't giving anything in simulation
        #sub = rospy.Subscriber("/"+self.robot_name+"/gen3_lite_2f_gripper_controller/gripper_cmd/feedback", InterconnectCyclic_Feedback, self.read_gripper_pos)

        #but rostopic echo /my_gen3_lite/base_feedback did
        sub = rospy.Subscriber("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback, self.read_gripper_pos)

        req = SendGripperCommandRequest()
        finger = Finger()

        finger1 = command["gripper"]["finger"][0]

        finger.finger_identifier = 1 #finger1["fingerIdentifier"] #for now, hardcoded
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


    prevReach = { #default previous position
        "targetPose":{
            "x": 0.38117900490760803,
            "y": 0.0644112303853035,
            "z": 0.22991076111793518,
            "thetaX": -90.0,
            "thetaY": 0.0,
            "thetaZ": -90.0
        }
    }
    seq_cancelled = False

    #Looks in the rosparams for sequences (combined_sequences.json)
    # Searches for the sequence with the correct name, and then calls
    # sequence_for_loop to iterate over the steps in each sequence
    def execute_sequence(self, seq_name, restrict_horizontal=True):
        self.seq_cancelled = False
        seqIndex = -1
        i = 0
        # find the sequence in the sequence list
        for sequence in rospy.get_param("/sequences/sequence"):
            #rospy.loginfo(sequence["name"])
            if sequence["name"] == seq_name:
                seqIndex = i
                #rospy.loginfo(sequence["name"])
                break
            i += 1

        if seqIndex == -1:
            rospy.loginfo("Could not find the sequence \""+seq_name+"\"")
            # fail
            return False

        # now for each task in that sequence, execute the corresponding action
        tasks = rospy.get_param("/sequences/sequence")[seqIndex]["tasks"]
        return self.sequence_for_loop(tasks, restrict_horizontal)

    # Given a task list (ie. a sequence), this will iterate through the tasks
    #  and call go_to_cart_pose_relative, which moves the arm to a position
    #  relative to where it previously was (self.prevReach), or moves the gripper
    #  if seq_cancelled is ever true, the loop is stopped
    def sequence_for_loop(self, tasks, restrict_horizontal=True):
        rospy.loginfo("Sequence for loop")
        success = True
        self.prevReach = tasks[0]["action"]["reachPose"]
        for task in tasks[1:]: #skip the first task, its the relative position's origin
            rospy.loginfo("Do task")
            if self.seq_cancelled:
                rospy.loginfo("Sequence was cancelled")
                self.seq_cancelled = False
                break
            if "sendGripperCommand" in task["action"]:
                rospy.loginfo("Execute Gripper action")
                success &= self.go_to_gripper_pose(task["action"]["sendGripperCommand"])
                #success &= self.example_send_gripper_command(0.5)#task["action"]["sendGripperCommand"]["gripper"]["finger"][0]["value"])
            elif "reachPose" in task["action"]:
                rospy.loginfo("Execute ReachPose action")
                # success &= self.go_to_cart_pose(task["action"]["reachPose"])
                success &= self.go_to_cart_pose_relative(task["action"]["reachPose"], self.prevReach, restrict_horizontal)
                self.prevReach = task["action"]["reachPose"]
            elif "reachJointAngles" in task["action"]:
                rospy.loginfo("Execute Joint Angle Action")
                # nothing implemented for a jointAngle type of position
            else:
                success = False
                rospy.loginfo("Invalid Command Type")

        return success

    # if you want the robot to go to a single position (in combined_positions.json)
    # you can use this function
    def go_param_pose(self, index):
        reachPose = rospy.get_param("/positions/poses/pose")[index]["reachPose"]
        return self.go_to_cart_pose(reachPose)

    # reads a "reachPose" in a sequence and builds the appropriate
    # object for moving the arm to that position, then passes it to play_cartesian_trajectory
    # (the corresponding rosservice)
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

    # Takes a reachPose, and the last reachPose, calculates the relative motion that should
    # be applied, then calls the rosservice
    # calculating relative movement means that if the gripper is at a different
    # height compared to the actual recorded sequence, it will complete the motion at
    # this different height
    def go_to_cart_pose_relative(self, reachPose, prevReachPose, restrict_horizontal):
        rospy.loginfo("Go to cart pose relative")
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
        nparr_vec = vec
        nparr_prev_vec = prev_vec
        nparr_feedback = feedback_vec

        added_vec = vec_sub(nparr_vec, nparr_prev_vec) #make it a relative movement (default relative to retracted)
        added_vec = vec_add(added_vec, nparr_feedback) #add the rel to our actual previous position

        #spherical dist bound subtracts the ordinary origin, then we can add our starting point
        vec_res = spherical_dist_bound(added_vec,0.6)
        #if restrict_horizontal:
        #    vec_res = horizontal_dist_bound(vec_res)
        req = PlayCartesianTrajectoryRequest()
        req.input.target_pose.x = vec_res[0]
        req.input.target_pose.y = vec_res[1]
        req.input.target_pose.z = vec_res[2]
        req.input.target_pose.theta_x = (tPose["thetaX"] - prevTPose["thetaX"])+feedback.base.commanded_tool_pose_theta_x# + tPose["thetaX"]
        req.input.target_pose.theta_y = (tPose["thetaY"] - prevTPose["thetaY"])+feedback.base.commanded_tool_pose_theta_y# + tPose["thetaY"]
        req.input.target_pose.theta_z = (tPose["thetaZ"] - prevTPose["thetaZ"])+feedback.base.commanded_tool_pose_theta_z# + tPose["thetaZ"]

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

    # stops any sequence in progress, and breaks the sequence_for_loop
    def stop_seq(self):
        self.seq_cancelled = True
        self.send_stop_command()

    # checks the current gripper position, and chooses either the sequence grab or put back
    # based on whether the gripper is currently open or closed
    grabbed = False
    def grab_toggle(self):
        msg = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)
        self.read_gripper_pos(msg)
        if self.gripper_cur_pos > 0.95:#< 0.05: #closed
            self.execute_sequence("put_back_cart")
            self.grabbed = False
        else:
            self.execute_sequence("grab_cart")
            self.grabbed = True


    # The circular motion needs to be able to be started from any place around the outer
    # radius of the circle (this allows for resuming a circular motion sequence after it is
    # stopped). So this function finds the angle the gripper is at relative to the origin
    # and then, if farther from a certain distance of the origin, will return that angle
    # as the offset to use when starting the circular motion.
    def getCircleOffset(self, reverse=False, rad=0.2, origin=[0.40,0.06447,0.2299]):
        feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)

        h = feedback.base.commanded_tool_pose_z-origin[2]
        x = feedback.base.commanded_tool_pose_y-origin[1]
        rospy.loginfo("X,y")
        rospy.loginfo(x)
        rospy.loginfo(h)
        theta = 0
        #we need to take the absolute value of atan to assure that -x,+y values give positive angles
        #and ^^^ *-1 when x,-y to assure negative angles
        if h > 0:
            theta = math.atan(-1*abs(h/x))
        else:
            theta = math.atan(abs(h/x))
        offset = 0
        #if we are close to the outer radius of the circle, pick an offset based on that
        #if we are close to the origin, then don't use the theta offset
        dist = math.sqrt(h*h + x*x)
        if(dist > 0.05): #< 0.1
            offset = theta*57.2958
        rospy.loginfo("Offset, theta, dist is")
        rospy.loginfo(offset)
        rospy.loginfo(theta*57.2958)
        rospy.loginfo(dist)
        return offset

    #generate and execute a circular motion given a radius
    #wrist_fixed determines whether the gripper stays at the same angle or whether
    #the angle changes relative to where on the circle the arm is
    #wrist angle chooses the angle to put the gripper at
    def do_circle(self, radius=0.20, dir=1, wrist_fixed=False, wrist_angle=90, offset=0):
        reverse = radius < 0
        of = self.getCircleOffset(reverse,rad=radius)
        pts = gen_circle_axis(radius,12, dir=dir, offset=of)
        feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)

        #tasks[0]["action"]["reachPose"] << what it should look like
        tasks = [
            { #default previous position
                "action" :{
                    "reachPose":{
                        "targetPose": {
                            "x":feedback.base.commanded_tool_pose_x,
                            "y":feedback.base.commanded_tool_pose_y,
                            "z":feedback.base.commanded_tool_pose_z,
                            "thetaX": -90.0,
                            "thetaY": 0.0,
                            "thetaZ": -90.0
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
                "thetaX":-90.0,
                "thetaY":0.0,
                "thetaZ":-90.0
            }}}})
        self.sequence_for_loop(tasks, False)

    #1down =                                 = 0.11
    #base  = 0.22991076111793518            ~= 0.23
    #1up   = 0.2299+0.35-0.22991076111793518 = 0.35

    # Function for cycling the height of the arm between 3 different positions
    # does so by checking for the closest height, and then checking a lookup table
    # for the height above or below
    def cycle_height(self, down=False):
        feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)
        h = feedback.base.commanded_tool_pose_z
        #half way pts (x+0.6) 0.17, 0.29, 0.41
        possible_pts = [0.11, 0.23, 0.35, 0.47]
        stage = self.getClosestHeightStage(h)

        change_stage_by = 0
        if(down):
            change_stage_by = -1
        else:
            change_stage_by = 1

        if(down and stage == 0):
            stage = 2#3
        else:
            stage = (stage+change_stage_by) % 3#4

        position_vec = [
            feedback.base.commanded_tool_pose_x,
            feedback.base.commanded_tool_pose_y,
            possible_pts[stage],
        ]
        position_vec = spherical_dist_bound(position_vec,0.6)
        #position_vec = horizontal_dist_bound(position_vec)
        move_to = {"targetPose":{
            "x":position_vec[0],
            "y":position_vec[1],
            "z":position_vec[2],
            "thetaX":feedback.base.tool_pose_theta_x,
            "thetaY":feedback.base.tool_pose_theta_y,
            "thetaZ":feedback.base.tool_pose_theta_z
        }}#feedback

        self.go_to_cart_pose(move_to)

    # finds the closest index for the height lookup table in cycle_height()
    def getClosestHeightStage(self, h):
        stage = -1
        if (h <= 0.17):
            stage = 0
        elif (h <= 0.29):
            stage = 1
        # elif(h <= 0.41):
        #    stage = 2
        else:
            stage = 2  # 3
        return stage

    wrist_vertical = False
    def check_wrist_vertical(self):
        feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)
        return feedback.actuators[5].position < 45 or feedback.actuators[5].position > 315

    """
    The following is some code for handling controller inputs:
    -double_pressed: Given a button index, this function calls one of the various functions
        to move/control the arm based on the given button index, and the counter for if the button
        was double pressed or not. 
    -check_press: For a given button index and state, this function checks if a button was
        pressed, and if so, increments a number of presses counter. If it is the first press
        of a button, it starts a timer which when complete will call double_pressed
    -handle_controller: Set in main as the callback of the /joy node, meaning it receives
        the raw controller input data. At the moment, it just calls check_press for every
        button in the input data 
        
    Main modifications that could be made are:
        - The control scheme (change the actions of 'double_pressed()')
        - Modify handle_controller handle the joystick axis, sending that data to a new function,
            or treating axis values above a certain threshold as a button press
    """

    # can't take PS RT and LT, since jackal is turned on with those
    #currently: B=pick/put/poke, A=Screw motion, Y=Up/Down, X=Cancel, left stick down=default/go to home
    #2x select and 2x start= circular motions (different dirs), 1x select=rotate gripper
    # (share on ps controller)


    btnPressCount = 0
    btnIndLastPressed = -1
    lastBtnCheckWasPressed = False

    def double_pressed(self, btnIndex, event):
        #rospy.loginfo(self.btnPressCount)
        rospy.loginfo(self.btnIndLastPressed)
        btnEscape = 8#data.buttons[8] #share on ps, xbox btn on xbox
        btnOptions = 9#data.buttons[9] #stbl on xbox

        self.send_stop_command()
        if self.btnPressCount > 1 and btnIndex == self.btnIndLastPressed:
            #rospy.loginfo("Double Pressed")
            #if btnIndex == 0:
                #self.control_angle_vel() #test twisting
                #rospy.loginfo("Screw L")
                #self.screw_motion(-45)
                #self.send_joy_twist_command()
            if btnIndex == 1:
                self.execute_sequence("Poke_cart")
                #rospy.loginfo("Fix Elbow")
                grabbed = False
            #if btnIndex == 2:
                 #cancel the movement
                 #rospy.loginfo("Resume sequence")
                 #rospy.loginfo("Fix Elbow")

                 #self.stop_seq()
            if btnIndex == 3:
                rospy.loginfo("Go Down")
                self.cycle_height(True)

            if btnIndex == 6: #L
                rospy.loginfo("Do Circle L")
                self.do_circle(-0.15, -1,offset=0)
            if btnIndex == 7:#R
                rospy.loginfo("Do Circle R")
                self.do_circle(0.15)
            #reset to home, then ready position (useful if arm elbow gets in wrong place)
            if btnIndex == btnOptions:
                self.example_clear_faults()
                self.example_home_the_robot()
                self.go_param_pose(0)
                self.rotate_wrist(90, abs_angle=True)
        else:
            #rospy.loginfo("Single Pressed")
            # single press B for a gripper toggle
            if btnIndex == 0:
                rospy.loginfo("Screw R")
                #self.screw_motion(45)

                if self.check_wrist_vertical():#self.wrist_vertical:
                    self.execute_sequence("Screw Right Gripper Up")
                else:
                    self.execute_sequence("Screw Right")
            if btnIndex == 1:
                rospy.loginfo("do grab toggle")
                self.grab_toggle()
                #rospy.loginfo("Fix Elbow")
                #self.fix_elbow()
            if btnIndex == 2:
                 #cancel the movement
                 rospy.loginfo("Stop sequence")
                 self.stop_seq()
            if btnIndex == 3:
                rospy.loginfo("Go Up")
                #self.execute_sequence("up")
                self.cycle_height()

            if btnIndex == 6: #L
                #self.rotate_wrist(45)
                if self.wrist_vertical:
                    self.rotate_wrist(90, True)
                    self.wrist_vertical = False
                else:
                    self.rotate_wrist(0, True)
                    self.wrist_vertical = True
            if btnIndex == 7:

                #self.rotate_wrist(-45)
                rospy.loginfo("Do Circle Smooth")
                #self.do_circle_smooth(0.15)

            # go straight to the ready position
            if btnIndex == btnOptions:
                self.go_param_pose(0)
            if btnIndex == btnEscape:
                rospy.signal_shutdown("User Exited")

        self.btnPressCount = 0
        self.btnIndLastPressed = -1
        self.lastBtnCheckWasPressed = 0

    def check_press(self, btnIndex, btnState):
        if self.btnIndLastPressed == btnIndex:
            if self.lastBtnCheckWasPressed and btnState == 0: #on button up
                if self.btnPressCount == 0: #if this is the first press of a series
                    # make a callback "partial" func which has btnIndex filled out.
                    pressedCallback = partial(self.double_pressed, btnIndex)
                    #This will call the callback at the end of the timer, we determine then if we double pressed
                    rospy.Timer(rospy.Duration(0.2), pressedCallback, oneshot=True)
                self.btnPressCount += 1
            self.lastBtnCheckWasPressed = btnState == 1
        elif btnState == 1:
            self.btnIndLastPressed = btnIndex
            self.lastBtnCheckWasPressed = True
            self.btnPressCount = 0

    def handle_controller(self, data):
        for index, btn in enumerate(data.buttons):
            self.check_press(index, btn)
        #ax0 = data.axes[0]


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
            # Activate the action notifications
            success &= self.example_subscribe_to_a_robot_notification()

            #set the axis camera to a good default pan and tilt
            axis_cmd = Axis()
            axis_cmd.pan = 0.0
            axis_cmd.tilt = 5.0
            self.cmd.publish(axis_cmd)

            rospy.loginfo("go to home")
            self.example_home_the_robot()
            self.go_param_pose(0)

            # Set the reference frame to "Mixed"
            success &= self.example_set_cartesian_reference_frame()

            #open the gripper
            success &= self.example_send_gripper_command(0.0)
            self.gripper_closed = False

            #set the gripper rotation
            rospy.loginfo("rotate wrist")
            self.rotate_wrist(90, abs_angle=True)

            #check whether we passed the use_joy parameter as true or false
            if rospy.get_param("/use_joy")==True:
                #subscribe to the controller input and use it indefinitely
                rospy.loginfo("Using Joystick Input")
                rospy.Subscriber("/joy", Joy, self.handle_controller)
                rospy.spin()
            else:
                #if we dont have a controller, do a demo sequence
                rospy.loginfo("Doing Demo Sequence")
                #self.execute_sequence("Circle")
                #self.execute_sequence("Screw Right") #almost perfect

                #self.do_circle_smooth(-0.2, -1, offset=0)
                self.do_circle(0.2)
                self.rotate_wrist(0, abs_angle=True)

                #self.grab_toggle()
                #self.grab_toggle()
                #self.execute_sequence("Poke_cart")

                self.do_circle(0.2)
                feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)
                rospy.loginfo(feedback.base)
        if not success:
            rospy.logerr("The example encountered an error.")

if __name__ == "__main__":
    ex = Control()
    ex.main()
