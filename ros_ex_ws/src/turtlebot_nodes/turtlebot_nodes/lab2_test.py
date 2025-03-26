from irobot_create_msgs.msg import LightringLeds, AudioNote
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

class Lab2TestNode(Node):
    def __init__(self):
        super().__init__('lab2_test_node')

        # Cap velocity to 0.4 m/s
        self.vel_cap = 0.4

        self.obstacle = False

        # Subscribe to the velocity unfiltered commands
        self.vel_subscriber = self.create_subscription(Twist, '/robot1/cmd_vel_unfiltered', self.callback_vel, 10)
        self.vel_subscriber

        # Subscirbe to scan
        self.scan_sub = self.create_subscription(LaserScan, '/robot1/scan', self.callback_scan, 10)
        self.scan_sub

        # Publisher for LEDs
        self.led_publish = self.create_publisher(LightringLeds, '/robot1/cmd_lightring', qos_profile_sensor_data)

        # Velocity publisher
        self.vel_publish = self.create_publisher(Twist, '/robot1/cmd_vel', 10)

    # Callback for velocity sub
    def callback_vel(self, msg):
        robot_vel_fwd_unfiltered = msg.linear.x
        
        # If velocity is more than cap, set to cap
        if robot_vel_fwd_unfiltered > self.vel_cap:
            self.robot_vel = self.vel_cap
            self.get_logger().info('Too fast! Velocity is capped at "%s"' % str(self.robot_vel))
        else:
            self.robot_vel = robot_vel_fwd_unfiltered
            self.get_logger().info('Velocity is "%s"' % str(self.robot_vel))

        if self.obstacle:
            self.robot_vel = 0.0

        # Publish filtered velocity
        vel_msg = Twist()
        vel_msg.linear.x = self.robot_vel
        vel_msg.angular.z = msg.angular.z
        self.vel_publish.publish(vel_msg)

        # Publish LED to red if too fast, purple if obstacle
        light_msg = self.set_lightring_colors(self.robot_vel,self.obstacle)
        self.led_publish.publish(light_msg)


    # Callback for scan
    def callback_scan(self, msg):
        range_min = msg.range_min
        range_max = msg.range_max

        self.obstacle = False
        
        # front range is a:b (total range is 0:1080)
        a = 200
        b = 340

        for angle in range(a,b):
            scan_point = msg.ranges[angle]
            
            if (scan_point <= range_min) or (scan_point >= range_max):
                pass
            else:
                if scan_point <= 0.75:
                    # detect obstacle
                    self.obstacle = True
                    self.get_logger().info("Obstacle detected!")


    # Set led colors based on velocity
    def set_lightring_colors(self,vel,obstacle):
        lightring_msg = LightringLeds()
        lightring_msg.header.stamp = self.get_clock().now().to_msg()
        lightring_msg.override_system = True

        if vel < self.vel_cap:
            for i in range(6):
                lightring_msg.leds[i].red = 0
                lightring_msg.leds[i].blue = 0
                lightring_msg.leds[i].green = 255

        else:
            for i in range(6):
                lightring_msg.leds[i].red = 255
                lightring_msg.leds[i].blue = 0
                lightring_msg.leds[i].green = 0

        if obstacle:
            for i in range(6):
                lightring_msg.leds[i].red = 255
                lightring_msg.leds[i].blue = 255
                lightring_msg.leds[i].green = 0


        return lightring_msg

   

def main(args=None):
    rclpy.init(args=args)
    node = Lab2TestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
