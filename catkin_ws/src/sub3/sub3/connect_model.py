import rclpy
from rclpy.node import Node
from sub3.iot_udp import *
from ssafy_msgs.msg import TurtlebotStatus, EnviromentStatus
import time
import os
import socket
import threading
import struct
import binascii
import copy
import numpy as np
import cv2
import base64
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import CompressedImage, LaserScan

ex_scenario_object = {
    "light_livingroom_all": [["3375cd519eeb47dfa4988a24702dc577", "5f12d15a9962442cbbcd57b103fdd891"], "light_livingroom_all", "IDLE", "OFF", [-2.67386865615844, 7.1674485206604], [0., 0.]]  # 거실 전체
}


class connect_model(Node):

    def __init__(self):
        super().__init__('connect_model')

        # iot_udp 객체 생성
        self.iot = iot_udp()
        time.sleep(0.5)
        # envir_status 데이터
        self.envir = {"day": 0, "hour": 0, "minute": 0, "month": 0, "temperature": 0, "weather": ""}
        self.ebvir_sub = self.create_subscription(EnviromentStatus, '/envir_status', self.envir_callback, 10)
        # 이미지 Array 데이터
        self.subscription_img = self.create_subscription(CompressedImage, '/person/img', self.transImg_callback, 1)

        # 로봇의 좌표 subscribe로 받아오기
        self.cur_loc = [0., 0.]
        self.subscription = self.create_subscription(TurtlebotStatus, '/turtlebot_status', self.listener_callback, 10)

        # gaol_pose_radom 받기
        self.subscription_gaol_pose_random = self.create_subscription(PoseStamped, '/goal_pose_random', self.listener_callback_random, 10)

        # scenario_object 데이터
        # self.scenario_object = copy.deepcopy(ex_scenario_object)
        self.scenario_object = {}
        # 스캔된 데이터 목록
        self.scanned_reg_objs = {}
        self.scanned_new_objs = {}
        self.scanControl = False

        # object on/off 제어
        self.is_control_obj = False

        # mutex lock
        self.lock = threading.Lock()

        # goal_pos 재설정
        self.goal_pose_pub = self.create_publisher(PoseStamped, 'goal_pose', 1)

        # thief picture
        self.is_security_mode = None
        self.thiefPic = None
        self.img_header = None
        self.recTimeStamp = int(time.time())  # 도둑을 찍은 시간
        # base64로 인코딩된 배열
        self.encodeImgs = []
        self.picturIndex = 0

        # thread = threading.Thread(target=self.getCommand)
        # thread.daemon = True
        # thread.start()

    def getCommand(self):
        while True:
            print(f'1 : scan , 2 : reg , 3 : on , 4 : off , 5 : print list')
            cmd = int(input())
            if cmd == 1:
                self.makeScanList()
            elif cmd == 2:
                print("input obj uid")
                objuid = str(input())
                print("make obj name")
                objname = str(input())
                self.scenario_object[objname] = [
                    [objuid], 'IDLE', 'OFF', [0., 0.], [0., 0.]]
            elif cmd == 3:
                print("input obj name")
                objname = str(input())
                self.onObj(objname)
            elif cmd == 4:
                print("input obj name")
                objname = str(input())
                self.offObj(objname)
            elif cmd == 5:
                print(self.scenario_object)

    def envir_callback(self, msg):
        self.envir = msg

    # 방범 스위치 켰을 때 랜덤하게 이동
    def listener_callback_random(self, msg):
        if self.is_security_mode and not self.is_control_obj:
            tmp = PoseStamped()
            tmp.header.frame_id = 'map'
            tmp.pose.position.x = msg.pose.position.x
            tmp.pose.position.y = msg.pose.position.y
            tmp.pose.orientation.w = msg.pose.orientation.w
            self.goal_pose_pub.publish(tmp)

    # 계속 로봇 위치 받는 함수
    def listener_callback(self, msg):
        self.cur_loc[0] = msg.twist.angular.x
        self.cur_loc[1] = msg.twist.angular.y

    # 이미지 Array를 받아오기
    def transImg_callback(self, msg):
        # publish 확인 해서 사진이 있으면 변환 (opencv)
        # self.thiefPic = None 에 삽입
        # 헤더가 find/ nofind가 있음
        header = msg.header.frame_id
        if int(time.time()) - self.recTimeStamp > 10:
            if self.is_security_mode and header == 'find':  # 버튼 눌리고, 발견했고, 카운트 작동했을때
                self.recTimeStamp = int(time.time())
                # thiefPic에는 사진을 인코딩한 값을 넣는다
                self.thiefPic = np.frombuffer(msg.data, np.uint8)
                self.img_bgr = cv2.imdecode(self.thiefPic, cv2.IMREAD_COLOR)
                # 배열에 저장
                cv2.imwrite('C:\\img\\image'+str(self.picturIndex)+'.jpg',
                            self.img_bgr, [cv2.IMWRITE_JPEG_QUALITY, 50])
                readImg = open('C:\\img\\image' +
                               str(self.picturIndex)+'.jpg', 'rb')
                img_b64 = base64.b64encode(readImg.read())
                # 인코딩을 하고 배열에 저장 encodeImgs
                self.encodeImgs.append(str(img_b64))
                # 파일 구분하는 index값 증가
                self.picturIndex += 1

    # 스캔을 켰을 때
    def makeScanList(self):
        self.scanControl = True
        while self.scanControl:
            if self.iot.is_recv_data:
                self.lock.acquire()
                tmp = list(self.iot.recv_data)
                flag = True
                for skey in self.scenario_object.keys():
                    if tmp[0] in self.scenario_object[skey][0]:
                        self.scenario_object[skey][1] = tmp[1]
                        self.scenario_object[skey][2] = tmp[2]
                        self.scanned_reg_objs[skey] = copy.deepcopy(
                            self.scenario_object[skey])
                        flag = False
                if flag:
                    self.scanned_new_objs[tmp[0]] = tmp
                self.lock.release()
        self.scanControl = False
        self.scanned_reg_objs = {}
        self.scanned_new_objs = {}

    # 물체 켜기
    def onObj(self, name):
        try :
            obj = self.scenario_object[name]

            # 물체까지 움직이는 로직
            x = obj[4][0]
            y = obj[4][1]
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.pose.position.x = x
            goal_pose.pose.position.y = y
            goal_pose.pose.orientation.w = 1.0
            self.goal_pose_pub.publish(goal_pose)

            # 물체 조작
            st_timestamp = int(time.time())
            while self.is_control_obj:
                iot_loc = self.cur_loc
                time.sleep(0.5)
                if int(time.time()) - st_timestamp > 120:
                    self.is_control_obj = False
                    break
                if abs(iot_loc[0] - self.cur_loc[0]) + abs(iot_loc[1] - self.cur_loc[1]) < 0.5:
                    if abs(goal_pose.pose.position.x - self.cur_loc[0]) < 0.3 and abs(goal_pose.pose.position.y - self.cur_loc[1]) < 0.3:
                        break
            if self.is_control_obj :
                self.iot.all_procedures(obj[0][0], 'ON')
                obj[3] = 'ON'
        except KeyError:
            print(self.scenario_object)
            print("keyerror")


    # 물체 끄기
    def offObj(self, name):
        try :
            obj = self.scenario_object[name]

            # 물체까지 움직이는 로직
            x = obj[4][0]
            y = obj[4][1]
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.pose.position.x = x
            goal_pose.pose.position.y = y
            goal_pose.pose.orientation.w = 1.0
            self.goal_pose_pub.publish(goal_pose)

            # 물체 조작
            st_timestamp = int(time.time())
            while self.is_control_obj:
                iot_loc = self.cur_loc
                time.sleep(0.5)
                if int(time.time()) - st_timestamp > 120:
                    self.is_control_obj = False
                    break
                if abs(iot_loc[0] - self.cur_loc[0]) + abs(iot_loc[1] - self.cur_loc[1]) < 0.15:
                    if abs(goal_pose.pose.position.x - self.cur_loc[0]) < 0.25 and abs(goal_pose.pose.position.y - self.cur_loc[1]) < 0.25:
                        break
            if self.is_control_obj :
                self.iot.all_procedures(obj[0][0], 'OFF')
                obj[3] = 'OFF'
        except KeyError:
            print(self.scenario_object)
            print("keyerror")



def main(args=None):
    rclpy.init(args=None)
    conMod = connect_model()
    rclpy.spin(conMod)
    conMod.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
