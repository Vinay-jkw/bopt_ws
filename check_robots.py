#!/usr/bin/env python3
import subprocess
import sys
import time

def check_controllers(robot_ns):
    required_controllers = [
        "traction_joint_controller",
        "steering_joint_controller",
        "lift_joint_controller",
        "joint_state_broadcaster"
    ]
    
    cmd = f"ros2 control list_controllers -c {robot_ns}/controller_manager"
    print(f"  [Controllers] Checking controller_manager...", end='', flush=True)
    try:
        output = subprocess.check_output(cmd, shell=True, stderr=subprocess.STDOUT, text=True)
    except subprocess.CalledProcessError as e:
        print(f" FAILED\n    Could not query controllers: {e.output.strip()}")
        return False
        
    all_active = True
    for ctrl in required_controllers:
        idx = output.find(ctrl)
        if idx == -1 or "active" not in output[idx:output.find('\n', idx)]:
            print(f"\n    [X] {ctrl} is MISSING/INACTIVE. Attempting to fix...", end='', flush=True)
            fix_cmd = f"ros2 run controller_manager spawner {ctrl} -c {robot_ns}/controller_manager"
            try:
                subprocess.run(fix_cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)
                # verify again
                output2 = subprocess.check_output(cmd, shell=True, text=True)
                idx2 = output2.find(ctrl)
                if idx2 != -1 and "active" in output2[idx2:output2.find('\n', idx2)]:
                    print(" FIXED!")
                else:
                    print(" FAILED TO FIX")
                    all_active = False
            except Exception:
                print(" TIMED OUT")
                all_active = False
            
    if all_active:
        print(" OK")
    return all_active

def check_nodes(robot_ns):
    required_nodes = [
        f"{robot_ns}/amcl",
        f"{robot_ns}/mqtt_bridge",
        f"{robot_ns}/bopt_main_controller",
        f"{robot_ns}/bopt_nmpc_controller",
        f"{robot_ns}/robot_state_publisher"
    ]
    
    print(f"  [Nodes] Checking critical nodes...", end='', flush=True)
    try:
        output = subprocess.check_output("ros2 node list", shell=True, text=True).split('\n')
    except subprocess.CalledProcessError:
        print(" FAILED to list nodes")
        return False
        
    all_running = True
    for node in required_nodes:
        if node not in output:
            print(f"\n    [X] Node {node} is NOT RUNNING")
            all_running = False
            
    if all_running:
        print(" OK")
    return all_running

def check_topic(topic_name, timeout=10):
    print(f"  [Topic] {topic_name}...", end='', flush=True)
    cmd = f"timeout {timeout} ros2 topic echo --once {topic_name} > /dev/null"
    try:
        subprocess.check_call(cmd, shell=True, stderr=subprocess.DEVNULL)
        print(" OK")
        return True
    except subprocess.CalledProcessError:
        print(" FAILED (No messages received)")
        return False

def check_robot(robot_idx):
    robot_ns = f"/robot{robot_idx:03d}"
    print(f"\n========================================")
    print(f" Checking {robot_ns}")
    print(f"========================================")
    
    healthy = True
    
    # 1. Check Nodes
    if not check_nodes(robot_ns):
        healthy = False
        
    # 2. Check Controllers
    if not check_controllers(robot_ns):
        healthy = False
        
    # 3. Check Topics
    topics_to_check = [
        f"{robot_ns}/odom",
        f"{robot_ns}/amcl_pose",
        f"{robot_ns}/lidar/front/scan",
        f"{robot_ns}/lidar/top3dl/scan"
    ]
    
    for topic in topics_to_check:
        if not check_topic(topic):
            healthy = False
            
    if healthy:
        print(f"\n✅ {robot_ns} IS FULLY READY FOR WORKFLOW!")
    else:
        print(f"\n❌ {robot_ns} HAS ISSUES!")
        
    return healthy

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 check_robots.py <number_of_robots>")
        print("Example: python3 check_robots.py 4")
        sys.exit(1)
        
    num_robots = int(sys.argv[1])
    all_good = True
    
    for i in range(1, num_robots + 1):
        if not check_robot(i):
            all_good = False
            
    print("\n========================================")
    if all_good:
        print("🏆 ALL ROBOTS ARE HEALTHY! You can start the workflows.")
    else:
        print("⚠️  SOME ROBOTS FAILED CHECKS! Do not start workflows yet.")
        print("Try restarting the launch file or fixing the missing controllers.")
    print("========================================\n")
