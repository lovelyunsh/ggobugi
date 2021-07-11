# import rclpy
# from rclpy.node import Node 
import time
import os
import socket
import threading
import struct
import binascii

# iot_udp 노드는 udp 통신을 이용해 iot로 부터 uid를 얻어 접속, 제어를 하는 노드입니다.
# sub1,2 에서는 ros 메시지를 이용해 쉽게 제어했지만, advanced iot 제어에서는 정의된 통신프로토콜을 보고 iot와 직접 데이터를 주고 받는 형식으로 제어하게 됩니다.
# 통신 프로토콜은 시뮬레이터 매뉴얼을 참조해주세요.


# 노드 로직 순서
# 1. 통신 소켓 생성
# 2. 멀티스레드를 이용한 데이터 수신
# 3. 수신 데이터 파싱
# 4. 데이터 송신 함수
# 5. 사용자 메뉴 생성 
# 6. iot scan 
# 7. iot connect
# 8. iot control

## 통신프로토콜에 필요한 데이터입니다. 명세서에 제어, 상태 프로토콜을 참조하세요. 
params_status = {
    (0xa,0x25 ) : "IDLE" ,
    (0xb,0x31 ) : "CONNECTION",
    (0xc,0x51) : "CONNECTION_LOST" ,
    (0xb,0x37) : "ON",
    (0xa,0x70) : "OFF",
    (0xc,0x44) : "ERROR"
}


params_control_cmd= {
    "TRY_TO_CONNECT" : (0xb,0x31 )  ,
    "SWITCH_ON" : (0xb,0x37 ) ,
    "SWITCH_OFF" : (0xa,0x70),
    "RESET" : (0xb,0x25) ,
    "DISCONNECT" : (0x00,0x25) 
}



        


# class iot_udp(Node):
class iot_udp:

    def __init__(self):
        # super().__init__('iot_udp')
        self.ip='127.0.0.1'
        self.port=7502
        self.send_port=7401
        
        self.testdata = "iot_udp"

        # 로직 1. 통신 소켓 생성
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        recv_address = (self.ip,self.port)
        self.sock.bind(recv_address)
        self.data_size=65535 
        self.parsed_data=[]
        
        # 로직 2. 멀티스레드를 이용한 데이터 수신
        thread = threading.Thread(target=self.recv_udp_data)
        thread.daemon = True 
        thread.start() 

        self.is_recv_data = False
        self.is_timeout = True

        # mutex lock
        self.lock = threading.Lock()
        
        # while True:
        #     # 로직 5. 사용자 메뉴 생성
        #     print('Select Menu [0: scan, 1: connect, 2:control, 3:disconnect, 4:all_procedures ] ')
        #     menu=int(input())

        #     if menu == 0:
        #         self.scan()
        #     elif menu == 1 :
        #         self.connect()
        #     elif menu == 2 :
        #         self.control()
        #     elif menu == 3 :
        #         self.disconnect()
        #     elif menu == 4 :
        #         self.all_procedures()
 
    def recv_udp_data(self):
        self.dataRenewalTimestamp = int(time.time())
        thread = threading.Thread(target=self.clearTimer)
        thread.daemon = True 
        thread.start() 
        while True:
            raw_data, sender = self.sock.recvfrom(self.data_size)
            self.data_parsing(raw_data)
            self.dataRenewalTimestamp = int(time.time())

    def clearTimer(self):
        while True:
            if self.is_recv_data :
                if int(time.time()) - self.dataRenewalTimestamp > 2:
                    self.lock.acquire()
                    self.is_recv_data = False
                    self.recv_data = ["", "", ""]
                    self.lock.release()
            else:
                time.sleep(1)

    def data_parsing(self,raw_data) :
        
        # 로직 3. 수신 데이터 파싱
        header=raw_data[:19].decode('utf-8')
        data_length=raw_data[19:23]
        aux_data=raw_data[23:35]

        if header == '#Appliances-Status$' and data_length[0] == 20:
            uid_pack=raw_data[35:51]
            uid=self.packet_to_uid(uid_pack)
        
            network_status='error'
            if raw_data[51] == 10 and raw_data[52] == 37 :
                network_status = 'IDLE'
            elif raw_data[51] == 11 and raw_data[52] == 49 :
                network_status = 'CONNECTION'
            elif raw_data[51] == 12 and raw_data[52] == 81 :
                network_status = 'CONNECTION_LOST'

            device_status='error'
            if raw_data[53] == 11 and raw_data[54] == 55 :
                device_status = 'ON'
            elif raw_data[53] == 10 and raw_data[54] == 112 :
                device_status = 'OFF'
            elif raw_data[53] == 12 and raw_data[54] == 68 :
                device_status = 'ERROR'
            
            self.lock.acquire()
            self.is_recv_data=True
            self.recv_data = [uid, network_status, device_status]
            self.lock.release()
        
            
    def send_data(self,uid,cmd):

        # 로직 4. 데이터 송신 함수 생성
        header=b"#Ctrl-command$"
        data_length=bytes([0x12, 0x0, 0x0, 0x0])
        aux_data=bytes([0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0])
        self.upper=header + data_length + aux_data
        self.tail=bytes([0x0D, 0x0A])

        uid_pack=self.uid_to_packet(uid)
        cmd_pack=bytes([cmd[0],cmd[1]])

        send_data=self.upper+uid_pack+cmd_pack+self.tail
        self.sock.sendto(send_data,(self.ip,self.send_port))
    

            
    def uid_to_packet(self,uid):
        uid_pack=binascii.unhexlify(uid)
        return uid_pack

        
    def packet_to_uid(self,packet):
        uid=""
        for data in packet:
            if len(hex(data)[2:4])==1:
                uid+="0"
            uid+=hex(data)[2:4]
        return uid


    def scan(self):
        
        print('SCANNING NOW.....')
        print('BACK TO MENU : Ctrl+ C')

        # 로직 6. iot scan
        while True:
            try:
                if self.is_recv_data==True :
                    print(f"uid={self.recv_data[0]}, network status={self.recv_data[1]}, device status={self.recv_data[2]}")
                time.sleep(0.5)
            except KeyboardInterrupt:
                print()
                break

        print("SCAN FIN")
            
    
    def timeouttimer(self, lim):
        self.is_timeout = False
        st_timestamp = int(time.time())
        while True:
            if int(time.time()) - st_timestamp > lim:
                break
        self.is_timeout = True    
        

    def connect(self, uid):
        
        thread = threading.Thread(target=self.timeouttimer, args=[5])
        thread.daemon = True 
        thread.start() 

        # 로직 7. iot connect
        while self.is_recv_data == True:
            if self.is_timeout:
                break
            tmp = list(self.recv_data)
            if tmp[0] == uid :
                if tmp[1] == 'CONNECTION':
                    break
                if tmp[1] == 'CONNECTION_LOST' :
                    self.send_data(uid, params_control_cmd["RESET"])
                elif tmp[1] == 'CONNECTION' or tmp[1] == 'IDLE' :
                    self.send_data(uid, params_control_cmd["TRY_TO_CONNECT"])
                else :
                    print('network status error')
                    break


    
    def control(self, uid, cmd):

        thread = threading.Thread(target=self.timeouttimer, args=[5])
        thread.daemon = True 
        thread.start()

        # 로직 8. iot control
        while self.is_recv_data == True:
            if self.is_timeout :
                break
            tmp = list(self.recv_data)
            if tmp[0] == uid:
                if tmp[2] == cmd:
                    break
                if cmd == 'ON' :
                    self.send_data(uid, params_control_cmd["SWITCH_ON"])
                elif cmd == 'OFF' :
                    self.send_data(uid, params_control_cmd["SWITCH_OFF"])
                else :
                    print('cmd error')
                    break



    def disconnect(self, uid):

        thread = threading.Thread(target=self.timeouttimer, args=[5])
        thread.daemon = True 
        thread.start()

        while self.is_recv_data == True:
            if self.is_timeout:
                break
            tmp = list(self.recv_data)
            if tmp[0] == uid:
                if tmp[1] != "CONNECTION":
                    break
                self.send_data(uid, params_control_cmd["DISCONNECT"])
        


    def all_procedures(self, uid, cmd):
        self.connect(uid)
        time.sleep(0.5)
        self.control(uid, cmd)
        time.sleep(0.5)
        self.disconnect(uid)


           
    def __del__(self):
        self.sock.close()
        print('del')



# def main(args=None):
#     rclpy.init(args=args)
#     iot = iot_udp()
#     rclpy.spin(iot)
#     iot.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
