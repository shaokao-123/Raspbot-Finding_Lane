import PID_Control
import Camera_init
from Car_Control import Car
import cv2



def cleanup():
    #清理资源
    Car.Car_Stop()
    picam2.stop()
    cv2.destroyAllWindows()
    print("资源清理完成")

#开启摄像头
picam2=Camera_init.Start()

#初始化小车
car=Car()

while 1:
    #获取图像
    frame=picam2.capture_array()
    
    #图像处理
    
    
    #中线检测
    
    
    #转向调节
    PID_Control.PID_Turn(left_x,right_x,offsets,left_lane_sum,right_lane_sum)
    

#释放
cleanup()