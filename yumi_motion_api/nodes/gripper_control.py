#!/usr/bin/env python

import rospy
from std_srvs.srv import EmptyResponse
from yumi_motion_api.srv import GripperControl
from abb_rapid_sm_addin_msgs.srv import SetSGCommand
from abb_robot_msgs.srv import TriggerWithResultCode

def handle_gripper_control(req):
    # Extract parameters from the request
    side = req.side
    action = req.action

    # Simulate controlling the gripper
    rospy.loginfo("Controlling gripper: %s %s" % (side, action))

    # Call the service to set the gripper command
    rospy.wait_for_service('/yumi/rws/sm_addin/set_sg_command')
    try:
        set_command_proxy = rospy.ServiceProxy('/yumi/rws/sm_addin/set_sg_command', SetSGCommand)
        if side == "left":
            arm = "T_ROB_L"
        elif side == "right":
            arm = "T_ROB_R"
        if action == "open":
            command = 7
        elif action == "close":
            command = 6
        response = set_command_proxy(arm, command, 0.0)  # Assuming left gripper

    except rospy.ServiceException as e:
        rospy.logerr("Failed to call set_sg_command service: %s" % e)
        return EmptyResponse()

    # Call the service to run the gripper routine
    rospy.wait_for_service('/yumi/rws/sm_addin/run_sg_routine')
    try:
        run_routine_proxy = rospy.ServiceProxy('/yumi/rws/sm_addin/run_sg_routine', TriggerWithResultCode)
        response = run_routine_proxy()
    except rospy.ServiceException as e:
        rospy.logerr("Failed to call run_sg_routine service: %s" % e)
        return EmptyResponse()

    return []

def handle_calibrate_grippers(req):
    # Simulate calibrating and initializing the grippers
    rospy.loginfo("Calibrating and initializing grippers")

    # Replace this with your actual gripper calibration and initialization logic
    # For demonstration purposes, it just logs the action
    return EmptyResponse()

if __name__ == "__main__":
    rospy.init_node('gripper_control_server')
    rospy.Service('gripper_control', GripperControl, handle_gripper_control)
    rospy.Service('calibrate_grippers', TriggerWithResultCode, handle_calibrate_grippers)
    rospy.loginfo("Ready to control gripper and calibrate grippers.")
    rospy.spin()
