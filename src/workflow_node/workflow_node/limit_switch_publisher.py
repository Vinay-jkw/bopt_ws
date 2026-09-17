import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

class LimitSwitchOdotPublisher(Node):
    def __init__(self):
        super().__init__('limit_switch_odot_publisher')
        
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        
        self.publisher = self.create_publisher(String, '/byd/can_odot_data', 10)
        
        # We can also use a timer to constantly publish the state if needed,
        # but publishing on every joint_states message is also fine.
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.limit_switch_triggered = False
        
        # Threshold for considering the limit switch triggered. 
        # You may need to adjust this depending on the exact URDF behavior.
        self.threshold = 0.10
        
    def joint_states_callback(self, msg: JointState):
        try:
            if 'limit_switch_bar_joint' in msg.name:
                idx = msg.name.index('limit_switch_bar_joint')
                pos = msg.position[idx]
                
                # Check if position exceeds the threshold
                if abs(pos) > self.threshold:
                    self.limit_switch_triggered = True
                else:
                    self.limit_switch_triggered = False
        except ValueError:
            pass
            
    def timer_callback(self):
        # Format expected by odot_state.py: "some_prefix <sensor_byte>"
        # parts = msg.data.split(' ') -> parts[1] is the sensor byte string
        # sensor_data[0] is back_sensor_byte
        # sensor_data[-1] is front_sensor_byte
        msg = String()
        
        if self.limit_switch_triggered:
            # 11 means both back and front sensors are '1' (triggered)
            # Adjust if you only want the back sensor triggered ('10')
            msg.data = "limit_switch 11"
        else:
            # 00 means neither is triggered
            msg.data = "limit_switch 00"
            
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LimitSwitchOdotPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
