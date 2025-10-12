import cv2
import numpy as np
import time
import math
import smbus
from picamera2 import Picamera2

# 小车控制相关的全局变量
_device = None
_addr = 0x16

def car_init():
    """初始化小车"""
    global _device, _addr
    try:
        _device = smbus.SMBus(1)
        print("小车初始化成功")
    except Exception as e:
        print(f'I2C初始化错误: {e}')

def write_u8(reg, data):
    """向指定寄存器写入单个字节数据"""
    global _device, _addr
    try:
        _device.write_byte_data(_addr, reg, data)
    except Exception as e:
        print(f'write_u8 I2C error: {e}')

def write_reg(reg):
    """写入寄存器（不带数据）"""
    global _device, _addr
    try:
        _device.write_byte(_addr, reg)
    except Exception as e:
        print(f'write_reg I2C error: {e}')

def write_array(reg, data):
    """向指定寄存器写入数据数组"""
    global _device, _addr
    try:
        _device.write_i2c_block_data(_addr, reg, data)
    except Exception as e:
        print(f'write_array I2C error: {e}')

def Ctrl_Car(l_dir, l_speed, r_dir, r_speed):
    """直接控制左右电机方向和速度"""
    try:
        reg = 0x01
        data = [l_dir, l_speed, r_dir, r_speed]
        write_array(reg, data)
    except Exception as e:
        print(f'Ctrl_Car I2C error: {e}')

def Control_Car(speed1, speed2):
    """智能控制小车，根据速度正负自动判断方向"""
    try:
        if speed1 < 0:
            dir1 = 0
        else:
            dir1 = 1
        if speed2 < 0:
            dir2 = 0
        else:
            dir2 = 1 
        
        Ctrl_Car(dir1, int(math.fabs(speed1)), dir2, int(math.fabs(speed2)))
    except Exception as e:
        print(f'Control_Car I2C error: {e}')

def Car_Run(speed1, speed2):
    """小车前进"""
    try:
        Ctrl_Car(1, speed1, 1, speed2)
    except Exception as e:
        print(f'Car_Run I2C error: {e}')

def Car_Stop():
    """小车停止"""
    try:
        reg = 0x02
        write_u8(reg, 0x00)
        print("小车停止")
    except Exception as e:
        print(f'Car_Stop I2C error: {e}')

def Car_Back(speed1, speed2):
    """小车后退"""
    try:
        Ctrl_Car(0, speed1, 0, speed2)
    except Exception as e:
        print(f'Car_Back I2C error: {e}')

def Car_Left(speed1, speed2):
    """小车左转 - 左轮慢，右轮快"""
    try:
        Ctrl_Car(0, speed1, 1, speed2)
    except Exception as e:
        print(f'Car_Left I2C error: {e}')

def Car_Right(speed1, speed2):
    """小车右转 - 左轮快，右轮慢"""
    try:
        Ctrl_Car(1, speed1, 1, speed2)
    except Exception as e:
        print(f'Car_Right I2C error: {e}')

def Car_Spin_Left(speed1, speed2):
    """小车原地左转"""
    try:
        Ctrl_Car(0, speed1, 1, speed2)
    except Exception as e:
        print(f'Car_Spin_Left I2C error: {e}')

def Car_Spin_Right(speed1, speed2):
    """小车原地右转"""
    try:
        Ctrl_Car(1, speed1, 0, speed2)
    except Exception as e:
        print(f'Car_Spin_Right I2C error: {e}')

class AutoLaneFollower:
    def __init__(self, image_width=320, image_height=240):
        # 初始化摄像头
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            main={"size": (image_width, image_height), "format": "RGB888"}
        )
        self.picam2.configure(config)
        
        self.width = image_width
        self.height = image_height
        
        # ROI参数
        self.roi_height = int(self.height * 0.4)
        self.roi_start = 0
        
        # 控制参数
        self.base_speed = 40        # 基础速度
        self.max_speed = 70         # 最大速度
        self.dead_zone = 20         # 死区范围
        self.max_offset = 100       # 最大偏移量
        self.kp = 1.0               # 比例控制系数
        
        # 启动摄像头
        self.picam2.start()
        time.sleep(2)
        
        # 初始化小车
        car_init()
        print("自动循迹系统初始化完成")
    
    def preprocess_image(self, frame):
        """图像预处理"""
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, binary = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY_INV)
        return binary
    
    def get_roi(self, binary_image):
        """获取感兴趣区域"""
        roi = binary_image[self.roi_start:self.roi_height, 0:self.width]
        return roi
    
    def find_lane_center(self, roi):
        """查找车道中线"""
        column_sums = np.sum(roi, axis=0) / 255
        nonzero_indices = np.where(column_sums > 6)[0]
        
        if len(nonzero_indices) == 0:
            return None
        
        left_bound = nonzero_indices[0]
        right_bound = nonzero_indices[-1]
        lane_center = (left_bound + right_bound) // 2
        
        full_center_x = lane_center
        full_center_y = self.roi_start + self.roi_height // 2
        
        return (full_center_x, full_center_y, left_bound, right_bound)
    
    def calculate_motor_speeds(self, offset):
        """根据偏移量计算左右电机速度"""
        # 限制偏移量范围
        offset = max(min(offset, self.max_offset), -self.max_offset)
        
        # 如果在死区内，直行
        if abs(offset) < self.dead_zone:
            left_speed = self.base_speed
            right_speed = self.base_speed
            action = "直行"
        else:
            # 比例控制：偏移量越大，速度差越大
            speed_difference = self.kp * abs(offset)
            
            if offset > 0:
                # 向右偏，需要向左转：左轮减速，右轮加速
                left_speed = 10
                right_speed = 50
                action = f"向左转"
            else:
                # 向左偏，需要向右转：左轮加速，右轮减速
                left_speed = min(self.base_speed + speed_difference, self.max_speed)
                right_speed = max(self.base_speed - speed_difference, 20)
                action = f"向右转"
        
        return left_speed, right_speed, action, offset
    
    def control_car_based_on_offset(self, offset, lane_detected):
        """根据偏移量控制小车"""
        if not lane_detected:
            # 未检测到车道，停止
            Car_Stop()
            return "停止", 0, 0, 0
        
        # 计算电机速度
        left_speed, right_speed, action, actual_offset = self.calculate_motor_speeds(offset)
        
        # 执行控制
        Control_Car(left_speed, right_speed)
        
        return action, left_speed, right_speed, actual_offset
    
    def visualize(self, frame, result, control_info):
        """可视化结果显示"""
        display_frame = frame.copy()
        
        # 绘制ROI区域
        cv2.rectangle(display_frame, (0, self.roi_start), 
                     (self.width, self.height), (0, 255, 0), 2)
        
        if result is not None:
            center_x, center_y, left_bound, right_bound = result
            
            # 绘制车道边界
            cv2.line(display_frame, 
                    (left_bound, self.roi_start),
                    (left_bound, self.height), (255, 0, 0), 2)
            cv2.line(display_frame,
                    (right_bound, self.roi_start),
                    (right_bound, self.height), (255, 0, 0), 2)
            
            # 绘制车道中心线
            cv2.line(display_frame,
                    (center_x, self.roi_start),
                    (center_x, self.height), (0, 0, 255), 3)
            
            # 绘制图像中心线
            image_center = self.width // 2
            cv2.line(display_frame,
                    (image_center, self.roi_start),
                    (image_center, self.height), (255, 255, 0), 2)
        
        # 显示控制信息
        action, left_speed, right_speed, offset = control_info
        
        cv2.putText(display_frame, f"动作: {action}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(display_frame, f"偏移: {offset}", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return display_frame
    
    def run_autonomous(self, show_display=True):
        """运行自主循迹"""
        print("开始自动循迹...")
        print("按 'q' 退出，按 's' 停止小车")
        
        try:
            while True:
                # 捕获图像
                frame = self.picam2.capture_array()
                
                # 处理图像
                binary = self.preprocess_image(frame)
                roi = self.get_roi(binary)
                result = self.find_lane_center(roi)
                
                # 计算偏移量和控制小车
                if result:
                    center_x, center_y, left_bound, right_bound = result
                    image_center = self.width // 2
                    offset = center_x - image_center
                    lane_detected = True
                else:
                    offset = 0
                    lane_detected = False
                
                # 控制小车
                control_info = self.control_car_based_on_offset(offset, lane_detected)
                action, left_speed, right_speed, actual_offset = control_info
                
                # 显示信息
                if show_display:
                    display_frame = self.visualize(frame, result, control_info)
                    cv2.imshow('Lane Following', cv2.cvtColor(display_frame, cv2.COLOR_RGB2BGR))
                
                # 打印控制信息
                print(f"动作: {action:6s} | 偏移: {int(actual_offset):4d} | 左轮: {int(left_speed):3d} | 右轮: {int(right_speed):3d}")
                
                # 检查按键
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('s'):
                    Car_Stop()
                
                time.sleep(0.05)
                
        except KeyboardInterrupt:
            print("程序中断")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """清理资源"""
        Car_Stop()
        self.picam2.stop()
        cv2.destroyAllWindows()
        print("资源清理完成")

if __name__ == "__main__":
    # 创建循迹对象
    follower = AutoLaneFollower(image_width=320, image_height=240)
    
    # 运行
    follower.run_autonomous(show_display=True)
    Car_Stop()