import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import pickle
import sys
import math

def euler_to_quaternion(yaw, pitch=0.0, roll=0.0):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw

class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer')
        # Publisher for the path message
        self.publisher_ = self.create_publisher(Path, 'visualized_pallet_path', 10)
        
        # Publish periodically so RViz can catch it when you open it
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        path_file_path = "/home/jkw/bopt_ws/src/workflow_node/constructed_rs_path_pp_control.pkl"
        with open(path_file_path, 'rb') as f:
            pp_data = pickle.load(f)
        
        self.raw_path = pp_data['pallet_path']
        self.get_logger().info(f"Loaded path with {len(self.raw_path)} points.")

    def timer_callback(self):
        path_msg = Path()
        # You may need to change 'map' to 'odom' or 'base_footprint' depending on your RViz fixed frame
        path_msg.header.frame_id = 'base_footprint'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for i in range(len(self.raw_path)):
            p = self.raw_path[i]
            
            # calculate yaw from this point to the next
            if i < len(self.raw_path) - 1:
                next_p = self.raw_path[i+1]
                yaw = math.atan2(next_p[1] - p[1], next_p[0] - p[0])
            elif i > 0:
                # copy from previous if it's the last point
                prev_p = self.raw_path[i-1]
                yaw = math.atan2(p[1] - prev_p[1], p[0] - prev_p[0])
            else:
                yaw = 0.0
                
            qx, qy, qz, qw = euler_to_quaternion(yaw)
            
            pose = PoseStamped()
            pose.header.frame_id = path_msg.header.frame_id
            pose.header.stamp = path_msg.header.stamp
            
            pose.pose.position.x = float(p[0])
            pose.pose.position.y = float(p[1])
            pose.pose.position.z = 0.0
            
            pose.pose.orientation.x = float(qx)
            pose.pose.orientation.y = float(qy)
            pose.pose.orientation.z = float(qz)
            pose.pose.orientation.w = float(qw)
            
            path_msg.poses.append(pose)
            
        self.publisher_.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    path_visualizer = PathVisualizer()
    try:
        rclpy.spin(path_visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        path_visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()