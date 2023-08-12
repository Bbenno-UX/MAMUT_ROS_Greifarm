import rclpy
from rclpy.node import Node
import time
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
typ=JointState
# class MinimalPublisher(Node):

#     def __init__(self):
#         super().__init__('minimal_publisher')
        

#         self.i = 0

#     def timer_callback(self):
        
#         #msg.data = 'Hello World: %d' % self.i
#         self.get_logger().info('Publishing: "%s"' % msgg.data)
#         self.i += 1
class MinimalSubpub(Node):

    def __init__(self):
        self.msgg = typ()
        super().__init__('joint_states')
        self.h=time.time()
        self.publisher_ = self.create_publisher(typ, 'topicc', 10)
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
    def listener_callback(self, msg):
        self.msgg=msg
        if time.time()-self.h>1:
            self.h=time.time()
            self.get_logger().info('I heard: "%s"' % msg.position)
    def timer_callback(self):
        #self.msgg=Twist()
        #self.msgg.linear.x=4.0
        #msg.data = 'Hello World: %d' % self.i
        self.get_logger().info('Publishing: "%s"' % self.msgg.position)
        self.publisher_.publish(self.msgg)
        self.get_logger().info("med")


def main(args=None):
    rclpy.init(args=args)

    minimal_subpub = MinimalSubpub()
    rclpy.spin(minimal_subpub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

    

    rclpy.spin(minimal_subpub)


    minimal_subpub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()