from geometry_msgs.msg import Twist, PoseWithCovarianceStamped

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

class PubPoseNode(Node):
    def __init__(self):
        super().__init__('pub_pose')

        # Subscribe to the velocity unfiltered commands
        self.pos_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/robot1/pose', self.callback_pos, 10)
        self.pos_subscriber
     

    # Callback for velocity sub
    def callback_pos(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        ang = msg.pose.pose.orientation.z
        pos_filt = "\n";
        pos_filt += " x: " + str(x) + "\n"
        pos_filt += " y: " + str(y) + "\n"
        pos_filt += " angle: " + str(ang) + "\n"
        self.get_logger().info(pos_filt)

   
   

def main(args=None):
    rclpy.init(args=args)
    node = PubPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
