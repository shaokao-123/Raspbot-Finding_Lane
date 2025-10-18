import PID_Control
import Camera
from Car_Control import Car
import cv2
import time
from Inverse_Perspective_Mapping import inverse_perspective_mapping



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
    frame=picam2.capture_array()
    
    #图像处理
    """透视变换"""
    birdseye_view=inverse_perspective_mapping(frame)
    
    
    #中线检测
    
    
    #转向调节
    PID_Control.PID_Turn(left_x,right_x,offsets,left_lane_sum=0,right_lane_sum=0)
    if cv2.waitKey(0) and 0xFF==ord('q'):
        break
    time.sleep(0.01)

#释放
cleanup()