import rclpy
from rclpy.node import Node
from sub3.connect_model import *
import socketio
import threading
import time


# client 는 socketio의 기본 API로 구성된 노드입니다. 서버와 연결을 시도해서 서버와 통신을 합니다.

# 각자의 서버 주소에 맞게 connect 함수 안을 바꿔주고, server 스켈레톤코드를 이용해 서비스를 하고 있다면, 연결이 됩니다.
# 버튼을 누르면 해당 키값에 맞는 함수들이 호출이 됩니다. 연결이 된 후에는 emit 함수를 이용해 서버로 키값과 데이터를 보냅니다.
# 이 노드는 AWS EC2에 구축한 서버와 통신만 하는 노드이고, ROS2와 연동하여 사용하면 스마트홈에서 얻은 데이터들을 서버로 보내고, 웹서버로부터의 명령을 ROS2로 전달할 수 있습니다.

# 노드 로직 순서
# 1. 클라이언트 소켓 생성
# 2. 데이터 수신 콜백함수
# 3. 서버 연결
# 4. 데이터 송신


def startConMod(conMod):
    rclpy.spin(conMod)
    conMod.destroy_node()
    rclpy.shutdown()


try:
    rclpy.init(args=None)
    conMod = connect_model()

    # 로직 1. 클라이언트 소켓 생성
    sio = socketio.Client()

    @sio.event
    def connect():
        conMod_thread = threading.Thread(target=startConMod, args=[conMod])
        conMod_thread.daemon = True
        conMod_thread.start()
        print('connection established')

    # 로직 2. 데이터 수신 콜백함수
    # time/weather/temperature refresh

    @sio.on("sendStateRefresh")
    def state_refresh(data):
        print('sendStateRefresh : ', data)
        sio.emit('sendTime', [
                 conMod.envir.month, conMod.envir.day, conMod.envir.hour, conMod.envir.minute])
        sio.emit('sendWeather', conMod.envir.weather)
        sio.emit('sendTemperature', conMod.envir.temperature)

    # scan on/off
    @sio.on("sendScanOff")
    def scan_off(data):
        print('sendScanOff : ', data)
        conMod.scanControl = False

    @sio.on('sendScanOn')
    def scan_on(data):
        print('sendScanOn : ', data)

        if conMod.scanControl:
            conMod.scanControl = False
            time.sleep(0.5)
        thread = threading.Thread(target=conMod.makeScanList)
        thread.daemon = True
        thread.start()
        time.sleep(0.5)

        start_timestamp = int(time.time())
        tmp_timestamp = start_timestamp
        cur_timestamp = start_timestamp
        while conMod.scanControl and cur_timestamp - start_timestamp < 20:
            cur_timestamp = int(time.time()) + 3
            if cur_timestamp - tmp_timestamp > 2:
                tmp_timestamp = cur_timestamp
                sio.emit('sendRegScannedObj', conMod.scanned_reg_objs)
                sio.emit('sendNewScannedObj', conMod.scanned_new_objs)

        if conMod.scanControl:
            sio.emit('sendRegScannedObj', conMod.scanned_reg_objs)
            sio.emit('sendNewScannedObj', conMod.scanned_new_objs)
        conMod.scanControl = False

    # get registed obj
    @sio.on('sendGetRegObj')
    def get_reg_obj(data):
        print('sendGetRegObj : ', data)
        sio.emit('sendRegistedObj', conMod.scenario_object)

    # obj on/off
    @sio.on('sendObjOn')
    def obj_on(data):
        print('sendObjOn : ', data)
        sio.emit('sendRegistedObj', conMod.scenario_object)
        if (not conMod.is_control_obj):
            conMod.is_control_obj = True
            conMod.onObj(data)
            conMod.is_control_obj = False
        sio.emit('sendRegistedObj', conMod.scenario_object)

    @sio.on('sendObjOff')
    def obj_off(data):
        print('sendObjOff : ', data)
        sio.emit('sendRegistedObj', conMod.scenario_object)
        if(not conMod.is_control_obj) :
            conMod.is_control_obj = True
            conMod.offObj(data)
            conMod.is_control_obj = False
        sio.emit('sendRegistedObj', conMod.scenario_object)

    # regist/remove obj
    @sio.on('sendRegistObj')
    def regist_obj(data):
        print('sendRegistObj : ', data)
        if data[1] in conMod.scenario_object.keys():
            conMod.lock.acquire()
            conMod.scenario_object[data[1]][0].append(data[0])
            conMod.lock.release()
        else:
            conMod.lock.acquire()
            print(data)
            conMod.scenario_object[data[1]] = [
                [data[0]], data[1], data[2], data[3], conMod.cur_loc.copy(), data[4]]
            print(conMod.scenario_object)
            conMod.lock.release()

    @sio.on('sendRemoveObj')
    def remove_obj(data):
        print('sendRemoveObj : ', data)
        if data in conMod.scenario_object.keys():
            conMod.lock.acquire()
            conMod.scenario_object.pop(data)
            conMod.lock.release()

    @sio.on('sendGetRocation')
    def get_rocation(data):
        # print('sendGetRocation : ', data)
        sio.emit('sendRocation', conMod.cur_loc)

    @sio.on('sendSecurityModeOn')
    def security_mode_on(data):
        print('sendSecurityModeOn : ', data)
        conMod.is_security_mode = True
        # 버튼이 ON됐다 => 배열 초기화, encodeImgs 초기화
        conMod.encodeImgs = []
        conMod.picturCnt = 0

    @sio.on('sendSecurityModeOff')
    def security_mode_off(data):
        print('sendSecurityModeOff : ', data)
        conMod.is_security_mode = False

    @sio.on('sendGetSecurityStatus')
    def security_mode_status(data):
        print('sendGetSecurityStatus : ', data)
        sio.emit('sendSecurityStatus', conMod.is_security_mode)

    @sio.on('sendGetThief')
    def get_thief_picture(data):
        print('sendGetThief : ', data)
        if conMod.is_security_mode == False:
            sio.emit('sendThief', ' '.join(conMod.encodeImgs))

    @sio.event
    def disconnect():
        print('disconnected from server')

    # 로직 3. 서버 연결
    sio.connect('http://j4c108.p.ssafy.io:8080/')
    # sio.connect('http://localhost:12001/')
    sio.sleep(1)

    # 로직 4. 데이터 송신 도둑 테스트
    # sio.wait()
    input()
except KeyboardInterrupt:
    exit()
