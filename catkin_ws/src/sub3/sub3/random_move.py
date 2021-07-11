import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry,OccupancyGrid
import time

# load_map 노드는 맵 데이터를 읽어서, 맵 상에서 점유영역(장애물) 근처에 로봇이 움직일 수 없는 영역을 설정하고 맵 데이터로 publish 해주는 노드입니다.
# 추 후 a_star 알고리즘에서 맵 데이터를 subscribe 해서 사용합니다.

# 노드 로직 순서
# 1. 맵 파라미터 설정
# 2. 맵 데이터 읽고, 2차원 행렬로 변환
# 3. 점유영역 근처 필터처리

class random_move(Node):

    def __init__(self):
        super().__init__('random_move')
        self.map_sub = self.create_subscription(OccupancyGrid,'map',self.map_callback,1)
        self.odom_sub = self.create_subscription(Odometry,'odom',self.odom_callback,1)
        self.goal_pub= self.create_publisher(PoseStamped, 'goal_pose_random', 1)
        time_period = 1
        self.timer = self.create_timer(time_period, self.timer_callback)
        
        self.map_msg=OccupancyGrid()
        self.odom_msg=Odometry()
        self.is_map=False
        self.is_odom=False
        self.is_found_path=False
        self.is_grid_update=False
        self.is_first_root=False

        self.map_size_x=350
        self.map_size_y=350
        self.map_resolution=0.05
        self.map_offset_x=-6-8.75
        self.map_offset_y=11-8.75

        self.goal = [0,0]
        self.count = 0
    
        self.GRIDSIZE=350


    def grid_update(self):
        self.is_grid_update=True
        map_to_grid=np.array(self.map_msg.data)
        self.grid=np.reshape(map_to_grid, (350,350), order='F')



    def pose_to_grid_cell(self,x,y):
        map_point_x=0
        map_point_y=0 
        map_point_x= int(( x - self.map_offset_x ) / self.map_resolution)
        map_point_y= int(( y - self.map_offset_y ) / self.map_resolution)
        return map_point_x,map_point_y
        


    def grid_cell_to_pose(self,grid_cell):
        x=(grid_cell[0]+(1/self.map_resolution)*self.map_offset_x)/20
        y=(grid_cell[1]+(1/self.map_resolution)*self.map_offset_y)/20
        return [x,y]
        


    def odom_callback(self,msg):
        self.is_odom=True
        self.odom_msg=msg

    def map_callback(self,msg):
        self.is_map=True
        self.map_msg=msg

    def timer_callback(self):
        if self.is_map ==True and self.is_odom==True  and self.is_first_root == False:
            if self.is_grid_update==False :
                self.grid_update()
            self.is_first_root = True
            x=self.odom_msg.pose.pose.position.x
            y=self.odom_msg.pose.pose.position.y

            ran_x, ran_y = 0, 0
            while True:
                ran_y = np.random.randint(350)
                ran_x = np.random.randint(350)
                if self.grid[ran_x][ran_y] < 50:
                    break
            waypoint_x, waypoint_y = self.grid_cell_to_pose([ran_x,ran_y])
            goal_pose = PoseStamped()
            goal_pose.header.frame_id='map'
            goal_pose.pose.position.x = waypoint_x
            goal_pose.pose.position.y = waypoint_y
            goal_pose.pose.orientation.w = 1.0
            self.goal = [waypoint_x,waypoint_y]
            self.goal_pub.publish(goal_pose)
            return
        if (self.count > 15) or (abs(self.goal[0]-self.odom_msg.pose.pose.position.x) < 1 and abs(self.goal[1]-self.odom_msg.pose.pose.position.y) < 1):
            self.is_first_root = False
            self.count = 0
            return
        self.count += 1

        
def main(args=None):
    rclpy.init(args=args)
    random_moving = random_move()
    rclpy.spin(random_moving)
    random_moving.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



       
   
