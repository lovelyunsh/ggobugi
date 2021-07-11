import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import Float32MultiArray
from ssafy_msgs.msg import BBox


# human detector node의 전체 로직 순서
# 로직 1 : 노드에 필요한 publisher, subscriber, descriptor, detector 정의
# 로직 2 : image binarization
# 로직 3 : human detection 실행 후 bounding box 출력
# 로직 4 : non maximum supresion으로 bounding box 정리
# 로직 5 : bbox를 ros msg 파일에 write
# 로직 6 : bbox를 원본 이미지에 draw
# 로직 7 : bbox 결과 show
# 로직 8 : bbox msg 송신




def non_maximum_supression(bboxes, threshold=0.37):

    """
    non maximum supression 로직
    로직 1 : bounding box 크기 역순으로 sort
    로직 2 : new_bboxes 리스트 정의 후 첫 bbox save
    로직 3 : 기존 bbox 리스트에 첫 bbox delete
    로직 4 : 두 bbox의 겹치는 영역을 구해서, 영역이 안 겹칠때 new_bbox로 save
    """

    # 로직 1 : bounding box 크기 역순으로 sort    
    bboxes = sorted(bboxes, key=lambda detections: detections[3],
            reverse=True)
    
    # 로직 2 : new_bboxes 리스트 정의 후 첫 bbox save
    new_bboxes=[]
    new_bboxes.append(bboxes[0])
    
    # 로직 3 : 기존 bbox 리스트에 첫 bbox delete
    bboxes.pop(0)

    for _, bbox in enumerate(bboxes):

        isNew = True
        for new_bbox in new_bboxes:

            x1_tl = bbox[0]
            x2_tl = new_bbox[0]
            x1_br = bbox[0] + bbox[2]
            x2_br = new_bbox[0] + new_bbox[2]
            y1_tl = bbox[1]
            y2_tl = new_bbox[1]
            y1_br = bbox[1] + bbox[3]
            y2_br = new_bbox[1] + new_bbox[3]
    
            # 로직 4 : 두 bbox의 겹치는 영역을 구해서, 영역이 안 겹칠때 new_bbox로 save
            x_len = (x1_br if x1_br > x2_br else x2_br) - (x1_tl if x1_tl < x2_tl else x2_tl)
            x_overlap = (bbox[2] + new_bbox[2]) - x_len
            x_overlap = x_overlap if x_overlap > 0 else 0
            y_len = (y1_br if y1_br > y2_br else y2_br) - (y1_tl if y1_tl < y2_tl else y2_tl)
            y_overlap = (bbox[3] + new_bbox[3]) - y_len
            y_overlap = y_overlap if y_overlap > 0 else 0
            overlap_area = x_overlap * y_overlap
            
            area_1 = bbox[2] * bbox[3]
            area_2 = new_bbox[2] * new_bbox[3]
            
            total_area = area_1 + area_2 - overlap_area
            overlap_area = overlap_area / float(total_area)
            # print(f"total_area {total_area} , overlap_area {overlap_area}")
            if overlap_area > threshold:
                isNew = False
                break

        if isNew :
            new_bboxes.append(bbox)
    return new_bboxes




class HumanDetector(Node):

    def __init__(self):
        super().__init__(node_name='human_detector')

        
        # 로직 1 : 노드에 필요한 publisher, subscriber, descriptor, detector, timer 정의

        self.subs_img = self.create_subscription(
            CompressedImage,
            '/image_jpeg/compressed',
            self.img_callback,
            1)

        self.img_bgr = None
        
        # self.timer_period = 0.03
        self.timer_period = 0.1

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.bbox_pub_ = self.create_publisher(BBox, '/bbox', 1)

        self.pedes_detector = cv2.HOGDescriptor()
        self.pedes_detector.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        self.able_to_pub = True

    def img_callback(self, msg):

        np_arr = np.frombuffer(msg.data, np.uint8)

        self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
  
    def detect_human(self, img_bgr):
    
        self.bbox_msg = BBox()

        # 로직 2 : image grayscale conversion
        img_pre = cv2.cvtColor(self.img_bgr, cv2.COLOR_BGR2GRAY)

        # 로직 3 : human detection 실행 후 bounding box 출력
        (rects_temp, _) = self.pedes_detector.detectMultiScale(img_pre)

        if len(rects_temp) != 0:
    
            # 로직 4 : non maximum supression으로 bounding box 정리

            xl, yl, wl, hl = [], [], [], []
            rects = non_maximum_supression(rects_temp)
    
            # 로직 5 : bbox를 ros msg 파일에 write

            ## 각 bbox의 center, width, height의 꼭지점들을 리스트에 넣어 놓고
            ## 메세지 내 x,y,w,h에 채워넣는 방식으로 하시면 됩니다.
           
            for (x,y,w,h) in rects:
    
                xl.append(int(x))
                yl.append(int(y))
                wl.append(int(w))
                hl.append(int(h))

            if self.able_to_pub:

                self.bbox_msg.num_bbox = len(rects)
                # if len(rects) < 2 and len(rects_temp) == 2 :
                #     print("low")
                # if len(rects) > 2 :
                #     print("high")

                obj_list = rects

                idxl = []
                for i in range(len(obj_list)) :
                    idxl.append(i)
                self.bbox_msg.idx_bbox = idxl

                self.bbox_msg.x = xl
                self.bbox_msg.y = yl
                self.bbox_msg.w = wl
                self.bbox_msg.h = hl

            # 로직 6 : bbox를 원본 이미지에 draw

            for (x,y,w,h) in rects:
                cv2.rectangle(img_bgr, (x,y),(x+w,y+h),(0,255,255), 2)

        else:
            # pass
            self.bbox_msg.num_bbox = len(rects_temp)


        ## 로직 7 : bbox 결과 show
        cv2.imshow("bbox_img", self.img_bgr)
        cv2.waitKey(1)

        """
        ## 일단 들어오는 원본 이미지를 보여준다.
        cv2.imshow("img", self.img_bgr)
        cv2.waitKey(1)

        """

    def timer_callback(self):

        if self.img_bgr is not None:

            self.detect_human(self.img_bgr)
            
            # 로직 8 : bbox msg 송신

            self.bbox_pub_.publish(self.bbox_msg)

        else:
            pass

def main(args=None):

    rclpy.init(args=args)

    hdetector = HumanDetector()

    rclpy.spin(hdetector)

if __name__ == '__main__':

    main()

