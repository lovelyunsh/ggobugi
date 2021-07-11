import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from squaternion import Quaternion
from nav_msgs.msg import Odometry,Path
from math import sqrt
import os

import sub2

# make_path 노드 설명
# 로봇의 위치(Odometry)를 받아서 매 0.1m 간격으로 x,y 좌표를 텍스트 파일에 기록하고, Path 메시지를 Publish 합니다.
# rviz에서 Path 메시지를 볼 수 있습니다. 경로가 잘 만들어지는지 rviz를 통해 확인해주세요.
# 기록을 해서 만드는 방법은 주행할 경로를 만드는 가장 쉬운 방법입니다. 생성된 텍스트 파일은 추 후에 텍스트 파일을 읽어서 Path 메시지를 publish 하는 노드(path_pub)에서 사용하고, 경로 추종알고리즘에서 사용 됩니다.


# 노드 로직 순서
# 1. 노드에 필요한 publisher, subscriber 생성
# 2. 저장할 경로 및 텍스트파일 이름을 정하고, 쓰기 모드로 열기
# 3. 콜백함수에서 처음 메시지가 들어오면 초기 위치를 저장
# 4. 이전 위치와 현재 위치의 거리차이 계산
# 5. 거리차이가 0.1m 이상일 때 위치를 path_msg.poses에 추가하고 publish
# 6. x,y 를 문자열로 바꾸고 x와 y 사이의 문자열은 /t 로 구분


class makePath(Node):

    def __init__(self):
        super().__init__('make_path')


        # 로직 1. 노드에 필요한 publisher, subscriber 생성      
        self.path_pub = self.create_publisher(Path, 'global_path', 10)
        self.subscription = self.create_subscription(Odometry,'/odom',self.listener_callback,10)


        '''
        로직 2. 저장할 경로 및 텍스트파일 이름을 정하고, 쓰기 모드로 열기

        빌드를 한 install에 있는 python 파일을 기준으로 src의 path파일 경로를 가져옵니다.
        '''

        now_path = os.path.abspath(__file__)
        full_path = now_path[:-48] + 'src/sub2/path/path.txt'
        self.f = open(full_path, 'w')
        
        # 원래 is_odom이 True로 되어 있었습니다만, 실행을 위한 변수 설정이었다고 생각되어 False로 변경하였습니다.
        self.is_odom=False
        ## 이전 위치를 저장할 변수입니다.
        self.prev_x=0.0
        self.prev_y=0.0

        self.path_msg=Path()
        self.path_msg.header.frame_id='map'



    def listener_callback(self,msg):
        # 아래의 print는 odom 노드에서 보낸 위치를 저장한 값입니다.
        # print('x : {} , y : {} '.format(msg.pose.pose.position.x,msg.pose.pose.position.y))
        if self.is_odom ==False :   
            '''
            로직 3. 콜백함수에서 처음 메시지가 들어오면 초기 위치를 저장
            
            초기 위치가 저장된 후에는 else을 돌며 x, y가 변화되어 길이가 0.1 만큼 이동할 때 동안 기다립니다.
            '''
            self.is_odom=True
            self.prev_x=msg.pose.pose.position.x
            self.prev_y=msg.pose.pose.position.y
            
            
        
        else :
            
            waypint_pose=PoseStamped()
            ## x,y 는 odom 메시지에서 받은 로봇의 현재 위치를 나타내는 변수입니다.
            x=msg.pose.pose.position.x
            y=msg.pose.pose.position.y
   

            '''
            로직 4. 이전 위치와 현재 위치의 거리차이 계산
            
            (테스트) 유클리디안 거리를 구하는 부분으로 x=2, y=2 이고, self.prev_x=0, self.prev_y=0 이라면 distance=2.82가 나와야합니다.
            
            유클리디안 거리란?

            유클리드 공간에서 작용하는 거리로 일상 속에서 거리 구하는 공식과 같습니다.
            x좌표와 y좌표의 차이 에서 피타고라스의 정리를 사용하여 거리를 구합니다.
            '''
            distance = sqrt(pow(x-self.prev_x, 2)+pow(y-self.prev_y, 2))
            


            if distance > 0.1 :
                '''
                로직 5. 거리차이가 위치보다 0.1m 이상일 때 위치를 path_msg.poses에 추가하고 publish

                현재의 x값과 y의 값이 저장되어 있으므로 움직였을 때 ( 거리가 0.1 넘게 차이나는 경우)
                만 찾아서 publish 시킵니다.
                '''
                waypint_pose.pose.position.x = x
                waypint_pose.pose.position.y = y
                waypint_pose.pose.orientation.w=1.0
                self.path_msg.poses.append(waypint_pose)
                self.path_pub.publish(self.path_msg)

                # 이동 했을 경우에만 출력합니다.
                print('x : {} , y : {} '.format(x,y))
                               

                '''
                로직 6. x,y 를 문자열로 바꾸고 x와 y 사이의 문자열은 /t 로 구분

                문자열끼리 더하면 문자열이 연결되어 새로운 문자열을 만드는 것을 활용하였습니다.

                새로운 문자열이 저장된 후에는 현재의 위치를 초기화된 위치로 바꿉니다.
                '''
                data= str(x) + '\t' + str(y) + '\n'
                self.f.write(data)
                

                self.prev_x=x
                self.prev_y=y
            
        
def main(args=None):
    rclpy.init(args=args)

    odom_based_make_path = makePath()

    rclpy.spin(odom_based_make_path)

    odom_based_make_path.f.close()
    odom_based_make_path.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
