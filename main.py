import control.PID_Control as PID_Control
from control.Car_Control import Car
import cv2
import time
import visual.image as image
from visual.ctype import detect_lane_center

from picamera2 import Picamera2

image_width=320
image_height=240

#启动摄像头
def init_camera():
    picam2=Picamera2()
    config=picam2.create_preview_configuration(main={"size":(image_width,image_height),"format":"RGB888"})
    picam2.configure(config)
    picam2.start()
    return picam2

def cleanup():
    #清理资源
    car.Car_Stop()
    picam2.stop()
    cv2.destroyAllWindows()
    print("资源清理完成")

#开启摄像头
picam2=init_camera()

#初始化小车
car=Car()

while 1:
    
    #获取图像
    frame = picam2.capture_array()
    #cv2.imshow('camera',frame)
    
    #图像处理
    birdseye_view = image.inverse_perspective(frame)
    binary = image.preprocess_image(birdseye_view,70)
    roi = image.get_roi(binary)
    cv2.imshow('binary',binary)
    #中线检测
    resized=cv2.resize(binary,(320,240))
    cv2.imshow('resized',resized)
    center_x,center_y=detect_lane_center(resized)
    offsets = -159 + center_x
    print(f"center_x:{center_x},offsets:{offsets}")
    #offsets=159-int((left_x+right_x)*0.5)
    
    #转向调节
    PID_Control.PID_Turn(offsets)
    
    if cv2.waitKey(1) and 0xFF==ord('q'):
        break
    time.sleep(0.001)
    
#释放
cleanup()
