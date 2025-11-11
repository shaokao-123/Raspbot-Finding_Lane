import PID_Ctrl
import Camera
from Car_Control import Car
import cv2
import time
import numpy as np
import image
#from test import inverse_perspective_mapping


def find_lane_center(roi):
    
    """查找车道中线"""
    column_sums = np.sum(roi, axis=0) / 255
    nonzero_indices = np.where(column_sums > 6)[0]
        
    if len(nonzero_indices) == 0:
        return None
        
    left_bound = nonzero_indices[0]
    right_bound = nonzero_indices[-1]
    lane_center = (left_bound + right_bound) // 2
        
    full_center_x = lane_center
    full_center_y = 0 + 96 // 2
        
    return (left_bound,right_bound,lane_center)

def cleanup():
    
    #清理资源
    car.Car_Stop()
    picam2.stop()
    picam2.close()
    cv2.destroyAllWindows()
    
    print("资源清理完成")


#开启摄像头
picam2=Camera.init_camera()

#初始化小车
car=Car()

while 1:
    
    #获取图像
    frame=picam2.capture_array()
    cv2.imshow('camera',frame)
    
    #图像处理
    #birdseye_view=image.inverse_perspective(frame)
    #cv2.imshow('birdseye_view',birdseye_view)
    binary=image.preprocess_image(frame,100)
    roi=image.get_roi(binary)
    cv2.imshow('binary',binary)
    
    #中线检测
    result=find_lane_center(roi)
    
    if not result:
        car.Car_Stop()
        
    if result:
        left_x,right_x,lane_center=result
        offsets = 159 - lane_center
        #转向调节
        PID_Ctrl.PID_Turn(offsets)  
    
    if cv2.waitKey(1)&0xFF==ord('q'):
        break
    time.sleep(0.001)
    
#释放
cleanup()
car.Car_Stop()
