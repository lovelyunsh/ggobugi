import rclpy
from rclpy.node import Node
from squaternion import Quaternion
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class path(Node):
    def __init__(self):
        super().__init__('path')
        self.global_path_pub = self.create_publisher(Path, 'global_path', 10)
        time_period = 0.1
        self.timer = self.create_timer(time_period,self.timer_callback)
        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = 'map'
        waypoints = [[0.0,0.0],
                     [1.0,0.0],
                     [1.0,1.0],
                     [2.0,2.0]]
        for waypoint in waypoints :
            point = PoseStamped()
            point.pose.position.x = waypoint[0]
            point.pose.position.y = waypoint[1]
            point.pose.orientation.w = 1.0
            self.global_path_msg.poses.append(point)
    
    def timer_callback(self):
              self.global_path_pub.publish(self.global_path_msg)


def main(args=None):
    rclpy.init(args=args)
    pat = path()
    rclpy.spin(pat)
    pat.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()