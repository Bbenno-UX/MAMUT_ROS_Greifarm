import rclpy
from rclpy.node import Node
import time
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
import numpy as np
from shapely.geometry import Polygon
typ=JointState
# class MinimalPublisher(Node):

#     def __init__(self):
#         super().__init__('minimal_publisher')
        

#         self.i = 0

#     def timer_callback(self):
        
#         #msg.data = 'Hello World: %d' % self.i
#         self.get_logger().info('Publishing: "%s"' % msgg.data)
#         self.i += 1
def Rects_aus_Linien(links,param=20,logger=None):
    koor_akt=np.array([0.0,0.0])
    Polys=[]
    koor_prev=np.copy(koor_akt)
    for i in links:
        koor_akt=i
        tangente=i/np.linalg.norm(i)
        normale=np.array([tangente[1],-tangente[0]])
        Polys.append(Polygon(
            ((koor_prev-tangente*param-normale*param),
            (koor_prev-tangente*param+normale*param),
            (koor_akt+tangente*param+normale*param),
            (koor_akt+tangente*param-normale*param))
        ))
        koor_prev=np.copy(koor_akt)
    return Polys


        
def VorwaertsKinematik(linkpunkte,winkel):
    endpunkte=np.copy(linkpunkte)
    winkel_akkum=0
    for ind,i in enumerate(winkel[1:4]):
        winkel_akkum+=i
        endpunkte[ind+1]*=np.exp(-winkel_akkum*1j)
        endpunkte[ind+1]+=endpunkte[ind]
    endpunkte=np.array([[i.real,i.imag] for i in endpunkte])
    return endpunkte    

class MinimalSubpub(Node):

    def __init__(self):
        self.msgg = typ()
        super().__init__('joint_states')
        self.h=time.time()
        self.linkpunkte_default=np.array([0+82j,24+128j,125+0j,125-2j])
        self.publisher_ = self.create_publisher(typ, 'topicc', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
    def listener_callback(self, msg):
        #msg.position=[-i for i in msg.position]
        self.msgg=msg
        
        if time.time()-self.h>2:
            self.h=time.time()
            self.get_logger().info('I heard: "%s"' % self.msgg.position)
        
    def timer_callback(self):
        #self.msgg=Twist()
        #self.msgg.linear.x=4.0
        #msg.data = 'Hello World: %d' % self.i
        geraden=VorwaertsKinematik(self.linkpunkte_default,self.msgg.position)
        rects=Rects_aus_Linien(geraden,logger=self.get_logger())
        bools=[
            rects[0].intersects(rects[3]),
            rects[0].intersects(rects[2]),
            rects[1].intersects(rects[3])
        ]
        if geraden[-1][1]<25 or geraden[-2][1]<25 or any(bools):
            self.get_logger().info('KOLLISION,KEINE NACHRICHT GESENDET')
        #self.get_logger().info('KOLLIS:'+str(bools))

        else:
            if time.time()-self.h>0.1:
                self.h=time.time()
                self.get_logger().info('Publishing: "%s"' % self.msgg.position)
            #self.msgg.position=[-i for i in self.msgg.position]
            self.publisher_.publish(self.msgg)
            #self.msgg.position=[-i for i in self.msgg.position]


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
