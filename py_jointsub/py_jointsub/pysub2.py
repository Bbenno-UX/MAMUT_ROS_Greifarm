import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import Int32
#from sensor_msgs.msg import JointState


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('joint_states')
        self.h=time.time()
        self.subscription = self.create_subscription(
            Int32,
            'pico_publisher',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        if time.time()-self.h>1:
            self.h=time.time()
            self.get_logger().info('I heard: "%s"' % msg.position)



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()