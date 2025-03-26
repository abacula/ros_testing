from irobot_create_msgs.msg import LightringLeds, AudioNote
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

class MapPubNode(Node):
    def __init__(self):
        super().__init__('map_pub')
        with open("file.txt", "w") as f:
            f.write("start\n")
        self.PI = 3.14159265358979323846
        self.obstacle = False
        self.x = 0
        self.y = 0
        self.ang = 0

        self.resoltuion = 0.05 # Default 5cm

        # Subscribe to the velocity unfiltered commands
        self.pos_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/robot1/pose', self.callback_pos, 10)
        self.pos_subscriber

        # Subscribe to the velocity unfiltered commands
        self.map_subscriber = self.create_subscription(OccupancyGrid, '/robot1/map', self.callback_map, 10)
        self.map_subscriber


        # Subscirbe to scan
        self.scan_sub = self.create_subscription(LaserScan, '/robot1/scan', self.callback_scan, 10)
        self.scan_sub

     

    # Callback for velocity sub
    def callback_pos(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.ang = msg.pose.pose.orientation.z
        pos_filt = "\n";
        pos_filt += " x: " + str(self.x) + "\n"
        pos_filt += " y: " + str(self.y) + "\n"
        pos_filt += " angle: " + str(self.ang) + "\n"
        # self.get_logger().info(pos_filt)


    def callback_map(self,msg):
        resolution = round(msg.info.resolution,3)
        # The origin of the map [m, m, rad].  This is the real-world pose of the  cell (0,0) in the map
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        origin_ang = msg.info.origin.orientation.z
        
        width = msg.info.width
        height = msg.info.height

        robot_x = round((self.x-origin_x)/resolution)
        robot_y = round((self.y-origin_y)/resolution)
        self.get_logger().info(str(robot_x) + ", " + str(robot_y) + ", " + str(self.ang*180/self.PI)) 

        robot_ang = (round(self.ang - origin_ang)*180/self.PI,4)

        occupancy_grid = msg.data

        print_grid = "VISUALIZATION:" + str(width) + "," + str (height) + ", " + str(robot_ang) + "\n"
        
        
        row = 0
        while row < height:
            col = 0
            print_grid += "|"
            while col < width:
                point = occupancy_grid[col + (row*width)]
                if (col == robot_x and row == robot_y):
                    print_grid += "R"
                elif point == -1:
                    print_grid += "_"
                elif point == 0:
                    print_grid += " "
                else:
                    print_grid += "x"
                col += 1
            print_grid += "|\n"
            row += 1
    
        # self.get_logger().info(print_grid)

        with open("file.txt", "a") as f:
            f.write(print_grid)

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
                    #self.get_logger().info("Obstacle detected!")

   

def main(args=None):
    rclpy.init(args=args)
    node = MapPubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
