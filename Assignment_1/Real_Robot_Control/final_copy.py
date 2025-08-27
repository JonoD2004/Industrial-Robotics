#!/usr/bin/env python3
import time
import roslibpy
import numpy as np

current_pos = None

def joint_state_cb(message):
    global current_pos
    # The JointState message contains 'position' array
    current_pos = list(message['position'])
    
def call_service(client, service_name):
    """Call a std_srvs/Trigger service and print the response."""
    service = roslibpy.Service(client, service_name, 'std_srvs/Trigger')
    request = roslibpy.ServiceRequest({})  # Trigger service takes no arguments

    print(f"[ROS] Calling service: {service_name}")
    result = service.call(request)
    print(f"[ROS] Response: success={result['success']}, message='{result['message']}'")


def move_ur_joint_positions(client, joint_positions, duration=5.0):
    global current_pos
    
    try:
        # Subscribe to joint states to get the current position
        listener = roslibpy.Topic(client, '/ur/joint_states', 'sensor_msgs/JointState')
        listener.subscribe(joint_state_cb)

        # Wait until we receive a joint state
        print("[ROS] Waiting for current joint state...")
        start_time = time.time()
        while current_pos is None and time.time() - start_time < 5.0:
            time.sleep(0.05)
        if current_pos is None:
            raise RuntimeError("No joint state received from /ur/joint_states")

        # Swap first and third joint positions
        current_pos[0], current_pos[2] = current_pos[2], current_pos[0]
        print(f"[ROS] Current joint positions: {current_pos}")

        # Build a JointTrajectory message for the scaled_pos_joint_traj_controller
        joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        trajectory_msg = {
            'joint_names': joint_names,
            'points': [
                {
                    'positions': current_pos,
                    'time_from_start': {'secs': 0, 'nsecs': 0}
                },
                {
                    'positions': joint_positions,
                    'time_from_start': {
                        'secs': int(duration),
                        'nsecs': int((duration - int(duration)) * 1e9)
                    }
                }
            ]
        }

        # Publish to the controller's /command topic
        topic = roslibpy.Topic(
            client,
            '/ur/scaled_pos_joint_traj_controller/command',
            'trajectory_msgs/JointTrajectory'
        )
        topic.advertise()
        topic.publish(roslibpy.Message(trajectory_msg))
        print("[ROS] Trajectory published.")

        # Wait for motion to complete
        time.sleep(duration + 1.0)

        topic.unadvertise()
        listener.unsubscribe()

    finally:
        print("[ROS] Finish")

if __name__ == '__main__':
    client = roslibpy.Ros(host='192.168.27.1', port=9090)  # Replace with your ROS bridge IP
    client.run()
    try:
        # --- Step 1: Move above pick position ---
        pick_pose = [0.0, -np.pi/4, np.pi/2, -np.pi/2, 0.0, 0.0]  # adjust to your setup
        move_ur_joint_positions(client, pick_pose, duration=5.0)

        # --- Step 2: Open gripper ---
        call_service(client, '/onrobot/open')
        time.sleep(2)

        # --- Step 3: Move down to grasp object ---
        pick_down_pose = [0.0, -np.pi/3, np.pi/2, -np.pi/2, 0.0, 0.0]  # lower Z
        move_ur_joint_positions(client, pick_down_pose, duration=5.0)

        # --- Step 4: Close gripper (pick) ---
        call_service(client, '/onrobot/close')
        time.sleep(2)

        # --- Step 5: Lift object back up ---
        move_ur_joint_positions(client, pick_pose, duration=5.0)

        # --- Step 6: Move to place location ---
        place_pose = [np.pi/4, -np.pi/4, np.pi/2, -np.pi/2, 0.0, 0.0]
        move_ur_joint_positions(client, place_pose, duration=5.0)

        # --- Step 7: Lower object ---
        place_down_pose = [np.pi/4, -np.pi/3, np.pi/2, -np.pi/2, 0.0, 0.0]
        move_ur_joint_positions(client, place_down_pose, duration=5.0)

        # --- Step 8: Open gripper (release) ---
        call_service(client, '/onrobot/open')
        time.sleep(2)

        # --- Step 9: Retract / return to home ---
        home_pose = [0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0]
        move_ur_joint_positions(client, home_pose, duration=5.0)

        print("[APP] Pick and place sequence completed.")
    except KeyboardInterrupt:
        print("\n[APP] Interrupted by user.")
