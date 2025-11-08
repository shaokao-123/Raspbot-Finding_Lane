import ctypes
import os
import numpy as np
from typing import Tuple, Optional

def detect_lane_center(binary_image: np.ndarray) -> Tuple[Optional[int], Optional[int]]:
    """
    
    使用ctypes调用C++车道检测程序
    
    """
    try:
        # 加载编译好的C++共享库
        lib_path = os.path.join(os.path.dirname(__file__), 'LaneCenterLocator2.so')
        lane_lib = ctypes.CDLL(lib_path)
        
        # 设置函数参数和返回类型
        lane_lib.calculate_lane_center.argtypes = [
            ctypes.POINTER(ctypes.c_int),  # image_data
            ctypes.c_int,                  # width
            ctypes.c_int,                  # height
            ctypes.POINTER(ctypes.c_int)   # result
        ]
        lane_lib.calculate_lane_center.restype = ctypes.c_int
        
        # 准备输入数据
        height, width = binary_image.shape
        flat_image = binary_image.flatten().astype(np.int32)
        image_ptr = flat_image.ctypes.data_as(ctypes.POINTER(ctypes.c_int))
        
        # 准备结果数组
        result_array = (ctypes.c_int * 2)(-1, -1)
        result_ptr = ctypes.cast(result_array, ctypes.POINTER(ctypes.c_int))
        
        # 调用C++函数
        success = lane_lib.calculate_lane_center(image_ptr, width, height, result_ptr)
        
        if success:
            center_x = result_array[0]
            center_y = result_array[1]
            return center_x, center_y
        else:
            return 1000, None
            
    except Exception as e:
        print(f"车道检测错误: {e}")
        return None, None
