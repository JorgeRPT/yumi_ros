import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from abb_robot_msgs.srv import GetIOSignal
from threading import Timer

class JointStateRerouter:
    def __init__(self):
        # Initialize global variables to store the latest joint positions and gripper positions
        self.gripper_l_position = 0.0
        self.gripper_r_position = 0.0
        self.latest_left_arm_positions = []
        self.latest_right_arm_positions = []
        self.continue_updating = True

        # Initialize ROS node
        rospy.init_node('joint_state_rerouter', anonymous=True)

        # Initialize publishers
        self.moveit_pub = rospy.Publisher('/yumi/egm/joint_state_controller/command', Float64MultiArray, queue_size=10)
        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

        # Initialize subscribers
        rospy.Subscriber('/move_group/fake_controller_joint_states', JointState, self.reroute_fake_callback)
        rospy.Subscriber('/yumi/rws/joint_states', JointState, self.joint_state_callback)

        # Start updating gripper position every 0.2 seconds
        ##self.update_gripper_position()

        # Set shutdown hook
        rospy.on_shutdown(self.shutdown_hook)

    def get_gripper_position(self):
        try:
            # Create a service proxy for getting the IO signal
            get_gripper_position_service = rospy.ServiceProxy('/yumi/rws/get_io_signal', GetIOSignal)

            # Get the left gripper position
            response_l = get_gripper_position_service(signal='hand_ActualPosition_L')
            self.gripper_l_position = float(response_l.value) / 10000.0

            # Get the right gripper position
            response_r = get_gripper_position_service(signal='hand_ActualPosition_R')
            self.gripper_r_position = float(response_r.value) / 10000.0

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def update_gripper_position(self):
        if not self.continue_updating:
            return
        self.get_gripper_position()
        # Call the function again after 0.2 seconds
        Timer(0.2, self.update_gripper_position).start()

    def reroute_fake_callback(self, data):
        transformed_data = Float64MultiArray()

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

        # Combine with latest positions if one arm's data is missing
        if not left_positions:
            print("Left arm data is missing. Using latest left arm positions.")
            print(f"latest_left_arm_positions: {self.latest_left_arm_positions}")
            left_positions = self.latest_left_arm_positions
            

        if not right_positions:
            print("Right arm data is missing. Using latest right arm positions.")
            print(f"latest_right_arm_positions: {self.latest_right_arm_positions}")
            right_positions = self.latest_right_arm_positions

        # Combining the left and right arm names and positions
        new_names = left_names + right_names
        new_positions = left_positions + right_positions

        # Separating the positions into left and right arm positions again for adjustments
        left_positions = new_positions[:len(new_positions)//2]
        right_positions = new_positions[len(new_positions)//2:]

        # Adjusting the 7th position of the left arm
        if len(left_positions) > 6:
            left_positions.insert(2, left_positions.pop(6))

        # Adjusting the 7th position of the right arm
        if len(right_positions) > 6:
            right_positions.insert(2, right_positions.pop(6))

        # Combining the left and right arm positions
        new_positions = left_positions + right_positions

        # Updating the message with the new positions
        transformed_data.data = new_positions

        # Log and publish the transformed data
        #rospy.loginfo(transformed_data)
        self.moveit_pub.publish(transformed_data)

    def joint_state_callback(self, data):
        joint_state = JointState()
        joint_state = data

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
        if len(left_positions) > 2:
            left_positions.insert(6, left_positions.pop(2))

        # Adjusting the 3rd position of the right arm
        if len(right_positions) > 2:
            right_positions.insert(6, right_positions.pop(2))

        # Store the latest positions !!!!! nao sei o porque de estarem trocados !!!!!!
        self.latest_left_arm_positions = right_positions
        self.latest_right_arm_positions = left_positions 
        
            # Print the latest positions to the console
        print("Joint Latest left arm positions: %s", self.latest_left_arm_positions)
        print("Joint Latest right arm positions: %s", self.latest_right_arm_positions)

        # Combining the left and right arm positions
        new_positions = left_positions + right_positions

        # Updating the message with the new positions
        joint_state.position = new_positions

        # Add gripper position to joint state
        joint_state.name.append('gripper_l_joint')
        joint_state.position.append(self.gripper_l_position)
        joint_state.name.append('gripper_r_joint')
        joint_state.position.append(self.gripper_r_position)

        # Publish the modified joint state
        self.joint_state_pub.publish(joint_state)

    def shutdown_hook(self):
        self.continue_updating = False

if __name__ == "__main__":
    # Create an instance of the JointStateRerouter class
    rerouter = JointStateRerouter()

    # Keep the node running
    rospy.spin()
