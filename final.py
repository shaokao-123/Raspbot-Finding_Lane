import PID_Control
import Camera
from Car_Control import Car
import cv2
import time
import image
from ctype import detect_lane_center

def cleanup():
    #清理资源
    car.Car_Stop()
    picam2.stop()
    cv2.destroyAllWindows()
    print("资源清理完成")

#开启摄像头
picam2=Camera.init_camera()

#初始化小车
car=Car()

while 1:
    
    #获取图像
    frame = picam2.capture_array()
    
    #图像处理
    birdseye_view = image.inverse_perspective(frame)
    binary = image.preprocess_image(birdseye_view)
    roi = image.get_roi(binary)
    
    #中线检测
    
    center_x,center_y=detect_lane_center(roi)
    
    #转向调节
    PID_Control.PID_Turn(center_x,320)
    
    if cv2.waitKey(1) and 0xFF==ord('q'):
        break
    time.sleep(0.001)
#释放
cleanup()
