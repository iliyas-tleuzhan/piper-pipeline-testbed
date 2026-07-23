import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--camera-only", action="store_true")
args = parser.parse_args()
print("Run later on HKU-CPS inside/alongside abot-piper-noetic: verify can0, ROS, /joint_states_single, /end_pose, MoveIt services, /table_camera topics, and http://localhost:8891/health.")
