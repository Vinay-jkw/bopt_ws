import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class FeedbackSubscriber(Node):
    def __init__(self):
        super().__init__('feedback_subscriber')
        self.subscription = self.create_subscription(
            String,
            '/load_transporter_feedback',
            self.listener_callback,
            10)
        self.subscription  

    def listener_callback(self, msg):
        self.get_logger().info('Received feedback: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    feedback_subscriber = FeedbackSubscriber()

    try:
        rclpy.spin(feedback_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        feedback_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
