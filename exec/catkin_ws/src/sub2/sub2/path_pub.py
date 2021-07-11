import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,PoseStamped
from squaternion import Quaternion
from nav_msgs.msg import Odometry,Path

from math import pi,cos,sin,sqrt
import tf2_ros
import os


# path_pub 노드는 make_path 노드에서 만든 텍스트 파일을 읽어와 전역 경로(/global_path)로 사용하고, 
# 전역 경로 중 로봇과 가장 가까운 포인트를 시작점으로 실제 로봇이 경로 추종에 사용하는 경로인 지역 경로(/local_path)를 만들어주는 노드입니다.


# 노드 로직 순서
# 1. publisher, subscriber 만들기
# 2. 만들어 놓은 경로 데이터를 읽기 모드로 open
# 3. 경로 데이터를 읽어서 Path 메시지에 데이터를 넣기
# 4. 주기마다 실행되는 타이머함수 생성, local_path_size 설정
# 5. global_path 중 로봇과 가장 가까운 포인트 계산
# 6. local_path 예외 처리
# 7. global_path 업데이트 주기 재설정


class pathPub(Node):

    def __init__(self):
        super().__init__('path_pub')

        # 로직 1. publisher, subscriber 만들기
        self.global_path_pub = self.create_publisher(Path, 'global_path', 10)
        self.local_path_pub = self.create_publisher(Path, 'local_path', 10)
        self.subscription = self.create_subscription(Odometry,'/odom',self.listener_callback,10)
        

        self.odom_msg=Odometry()
        self.is_odom=False

        ## 전역경로 메시지
        self.global_path_msg=Path()
        self.global_path_msg.header.frame_id='/map'


        '''
        로직 2. 만들어 놓은 경로 데이터를 읽기 모드로 open

        지난 번에 만든 make_path와 비슷하게 절대 경로를 생성했습니다.
        대신 make_path보다 파일명이 알바벳 1개가 짧으므로 -48 대신 -47로 문자열을 끊으면 됩니다.
        '''

        now_path = os.path.abspath(__file__)
        full_path = now_path[:-47] + 'src/sub2/path/path.txt'
        self.f = open(full_path, 'r')

     


        '''
        로직 3. 경로 데이터를 읽어서 Path 메시지에 데이터를 넣기

        파일의 x,y를 각각 읽어와서 global_pose 로 지정합니다.
        '''
        lines = self.f.readlines()
        for line in lines :
            tmp = line.split()
            read_pose = PoseStamped()
            read_pose.pose.position.x = float(tmp[0])
            read_pose.pose.position.y = float(tmp[1])
            read_pose.pose.orientation.w= 1.0
            self.global_path_msg.poses.append(read_pose)
        
        self.f.close()
        

        # 로직 4. 주기마다 실행되는 타이머함수 생성, local_path_size 설정
        time_period=0.02 
        self.timer = self.create_timer(time_period, self.timer_callback)
        self.local_path_size=30 

        self.count=0

    def listener_callback(self,msg):
        self.is_odom=True
        self.odom_msg=msg

    def timer_callback(self):
        if self.is_odom ==True:

            local_path_msg=Path()
            local_path_msg.header.frame_id='/map'
            
            x=self.odom_msg.pose.pose.position.x
            y=self.odom_msg.pose.pose.position.y
            print(x,y)
            current_waypoint=-1
            '''
            로직 5. global_path 중 로봇과 가장 가까운 포인트 계산

            가장 가까운 좌표를 찾아서 거리를 구합니다.

            그 때의 인덱스를 current_waypoint로 지정합니다.
            '''
            min_dis= float('inf')
            for i,waypoint in enumerate(self.global_path_msg.poses) :

                distance = sqrt(pow(x-waypoint.pose.position.x,2)+pow(y-waypoint.pose.position.y,2))
                if distance < min_dis :
                    min_dis= distance
                    current_waypoint = i
            
            
            '''
            로직 6. local_path 예외 처리

            current_waypoint 부터 local_path_size 개수만큼의 경로의 x, y의 좌표를 저장합니다.

            만약 현재위치에서 global_path_msg의 끝까지 local_path_size개수보다 적게 남았다면

            global_path_msg 끝까지 local_path_msg 로 지정합니다.
            '''
            if current_waypoint != -1 :
                if current_waypoint + self.local_path_size < len(self.global_path_msg.poses):
                    for num in range(current_waypoint, current_waypoint + self.local_path_size):
                        tmp_pose = PoseStamped()
                        tmp_pose.pose.position.x = self.global_path_msg.poses[num].pose.position.x
                        tmp_pose.pose.position.y = self.global_path_msg.poses[num].pose.position.y
                        tmp_pose.pose.orientation.w = 1.0
                        local_path_msg.poses.append(tmp_pose)
                  

                else :
                    for num in range(current_waypoint, len(self.global_path_msg.poses)):
                        tmp_pose = PoseStamped()
                        tmp_pose.pose.position.x = self.global_path_msg.poses[num].pose.position.x
                        tmp_pose.pose.position.y = self.global_path_msg.poses[num].pose.position.y
                        tmp_pose.pose.orientation.w = 1.0
                        local_path_msg.poses.append(tmp_pose)
                            
           
            self.local_path_pub.publish(local_path_msg)

        # 로직 7. global_path 업데이트 주기 재설정
        # 10번 실행할 때마다 global_path_pub를 1번 퍼블리싱하도록 주기를 늦춥니다.
        if self.count%10==0 :
            self.global_path_pub.publish(self.global_path_msg)
        self.count+=1

def main(args=None):
    rclpy.init(args=args)

    path_pub = pathPub()

    rclpy.spin(path_pub)

    path_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




       
   
