#!/usr/bin/env python

import rospy
from std_srvs.srv import TriggerResponse, Trigger
import subprocess

def run_command(command):
    process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE)
    output, error = process.communicate()
    #print("output:"+output,"\n")


commands = ["rosservice call /yumi/rws/stop_rapid", "rosservice call /yumi/rws/pp_to_main", "rosservice call /yumi/rws/start_rapid", "rosservice call /yumi/rws/sm_addin/start_egm_joint", "rosservice call /yumi/egm/controller_manager/switch_controller \"start_controllers: [joint_state_controller]\""]



def handle_egm_start(request):
    rospy.loginfo("starting egm session...")
    for command in commands:
        print(f"Running: {command}")
        run_command(command)
        print("Done.")
    
    return TriggerResponse(True, "Trigger successful")


def handle_egm_refresh(request):
    rospy.loginfo("refreshing egm session...")
    run_command("rosservice call /yumi/rws/sm_addin/start_egm_joint")
    print("Done")
    return TriggerResponse(True, "Trigger successful")


if __name__ == "__main__":
    rospy.init_node('egm_control_server')
    rospy.Service('start_egm_session', Trigger, handle_egm_start)
    rospy.Service('refresh_egm_session', Trigger, handle_egm_refresh)
    rospy.loginfo('Started EGM Server!')
    rospy.spin()



