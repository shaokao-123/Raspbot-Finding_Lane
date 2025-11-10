import PID_Control
import Camera
from Car_Control import Car
import cv2
import time
import image
#from ctype import detect_lane_center
import LCL2

def cleanup():
    #清理资源
    car.Car_Stop()
    picam2.stop()
    cv2.destroyAllWindows()
    print("资源清理完成")
    
def detect_lane_center(binary):
	lcl_detector = LCL2.LaneCalculator(width=binary.shape[1],height=binary.shape[0])
	center_x,center_y = lcl_detector.calculate_lane_center(binary).center_x,lcl_detector.calculate_lane_center(binary).center_y
	return center_x, center_y

#开启摄像头
picam2=Camera.init_camera()

#初始化小车
car=Car()

while 1:
    
    #获取图像
    frame = picam2.capture_array()
    
    #图像处理
    birdseye_view = image.inverse_perspective(frame)
    binary = image.preprocess_image(frame,100)
    #roi = image.get_roi(binary)
    cv2.imshow('binary',binary)
    
    #中线检测
    #resized=cv2.resize(binary,(320,240))
    center_x,center_y=detect_lane_center(binary)
    offsets=159-center_x
    #offsets=159-int((left_x+right_x)*0.5)
    print('offset: ',offsets)
    
    #转向调节
    PID_Control.PID_Turn(offsets)
    
    if cv2.waitKey(1) and 0xFF==ord('q'):
        break
    time.sleep(0.001)
    
#释放
cleanup()
