import subprocess

def run_command(command):
    process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE)
    output, error = process.communicate()
    print(output)

commands = ["rosservice call /yumi/rws/stop_rapid", "rosservice call /yumi/rws/pp_to_main", "rosservice call /yumi/rws/start_rapid", "rosservice call /yumi/rws/sm_addin/start_egm_joint", "rosservice call /yumi/egm/controller_manager/switch_controller \"start_controllers: [joint_state_controller]\""]

for command in commands:
    print(f"Running: {command}")
    run_command(command)
    print("Done.")
