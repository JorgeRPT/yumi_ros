#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from abb_robot_msgs.srv import GetIOSignal
from threading import Timer

# Global variable to store the gripper position
gripper_l_position = 0.0
gripper_r_position = 0.0
continue_updating = True

def get_gripper_position():
    global gripper_l_position
    global gripper_r_position
    try:
        get_gripper_position = rospy.ServiceProxy('/yumi/rws/get_io_signal', GetIOSignal)

        response_l = get_gripper_position(signal='hand_ActualPosition_L')
        #print(response_l.value)
        gripper_l_position = float(response_l.value) / 10000.0

        response_r = get_gripper_position(signal='hand_ActualPosition_R')
        gripper_r_position = float(response_r.value) / 10000.0


    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)


def update_gripper_position():
    global continue_updating
    if not continue_updating:
        return
    get_gripper_position()
    # Call the function again after 1 second
    Timer(0.2, update_gripper_position).start()


def reroute_fake_callback(data):
    transformed_data = Float64MultiArray()
    #rospy.loginfo(data)

   # Assuming your message is stored in a variable named 'message'

    # Extracting the original joint names and positions
    names = data.name
    positions = data.position

    # Separating the names and positions into left and right arm components
    left_names = []
    left_positions = []
    right_names = []
    right_positions = []

    for name, position in zip(names, positions):
        if name.endswith('_l'):
            left_names.append(name)
            left_positions.append(position)
        elif name.endswith('_r'):
            right_names.append(name)
            right_positions.append(position)

    # Combining the left and right arm names and positions
    new_names = left_names + right_names
    new_positions = left_positions + right_positions

    # Updating the message with the new names and positions
    data.name = new_names
    data.position = new_positions

    #rospy.loginfo(data)


    ############################################3
    positions = data.position

    # Separating the positions into left and right arm positions
    left_positions = positions[:len(positions)//2]
    right_positions = positions[len(positions)//2:]

    # Adjusting the 7th position of the left arm
    left_positions.insert(2, left_positions.pop(6))

    # Adjusting the 7th position of the right arm
    right_positions.insert(2, right_positions.pop(6))

    # Combining the left and right arm positions
    new_positions = left_positions + right_positions

    # Updating the message with the new positions
    transformed_data.data = new_positions

    #rospy.loginfo(transformed_data)

    rospy.loginfo(transformed_data)
    moveit_pub.publish(transformed_data)
    #rospy.loginfo("Published transformed message to /yumi/egm/joint_state_controller/command")





def joint_state_callback(data):
    joint_state = JointState()
    joint_state = data
    #print(joint_state)

    original_names = joint_state.name

    # Mapping from original names to new names
    new_names = []
    for name in original_names:
        if 'yumi_robr_joint' in name:
            new_names.append(name.replace('yumi_robr_joint', 'yumi_joint') + '_r')
        elif 'yumi_robl_joint' in name:
            new_names.append(name.replace('yumi_robl_joint', 'yumi_joint') + '_l')
        else:
            new_names.append(name)  # If the joint name doesn't match the expected format, keep it unchanged

    # Updating the message with the new names
    joint_state.name = new_names

    # Extracting the original positions
    positions = joint_state.position

    # Convert positions tuples to lists
    left_positions = list(positions[:len(positions)//2])
    right_positions = list(positions[len(positions)//2:])

    # Adjusting the 3rd position of the left arm
    left_positions.insert(6, left_positions.pop(2))

    # Adjusting the 3rd position of the right arm
    right_positions.insert(6, right_positions.pop(2))

    # Combining the left and right arm positions
    new_positions = left_positions + right_positions

    # Updating the message with the new positions
    joint_state.position = new_positions

    # Add gripper position to joint state
    joint_state.name.append('gripper_l_joint')
    joint_state.position.append(gripper_l_position)
    joint_state.name.append('gripper_r_joint')
    joint_state.position.append(gripper_r_position)


    #print(joint_state)
    joint_state_pub.publish(joint_state)



def shutdown_hook():
    global continue_updating
    continue_updating = False


if __name__ == "__main__":
    rospy.init_node('joint_state_rerouter', anonymous=True)



    moveit_pub = rospy.Publisher('/yumi/egm/joint_state_controller/command', Float64MultiArray, queue_size=10)

    joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)



    rospy.Subscriber('/move_group/fake_controller_joint_states', JointState, reroute_fake_callback)

    rospy.Subscriber('/yumi/rws/joint_states', JointState, joint_state_callback)

    # Start updating gripper position every second
    update_gripper_position()

    # Set shutdown hook
    rospy.on_shutdown(shutdown_hook)

    rospy.spin()
