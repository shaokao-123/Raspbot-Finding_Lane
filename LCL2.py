import cv2
import numpy as np

class LaneResult:
    def __init__(self):
        self.center_x = 0
        self.center_y = 0
        self.left_bound = 0
        self.right_bound = 0
        self.detected = False

class LaneCalculator:
    def __init__(self, width=320, height=240):
        self.image_width = width
        self.image_height = height
        self.min_white_pixels = 6

    def calculate_lane_center(self, binary_image):
        """
        从二值化图像计算车道中心位置
        :param binary_image: 二值化图像 (0=黑色, 255=白色)，类型为list[list[int]]
        :return: LaneResult对象
        """
        result = LaneResult()
        
        # 检查输入有效性
        if not binary_image.any():
            print("Invalid binary image input.")
            return result
        
        # 检测边缘
        edges = self.detect_edges_with_opencv(binary_image)
        cv2.imshow("Edges", edges)
        cv2.waitKey(0)
        # 提取车道边缘
        left_edges, right_edges = self.extract_lane_edges(edges)
        
        # 计算有效列
        valid_columns = []
        if left_edges and right_edges:
            for x in range(self.image_width):
                edge_count = 0
                for y in range(self.image_height):
                    if y < edges.shape[0] and x < edges.shape[1]:
                        if edges[y, x] == 255:
                            edge_count += 1
                if edge_count > self.min_white_pixels:
                    valid_columns.append(x)
        
        # 检查是否有有效列
        if not valid_columns:
            return result
        
        # 计算边界和中心
        left_bound = valid_columns[0]
        right_bound = valid_columns[-1]
        lane_center = self.calculate_edge_based_center(left_edges, right_edges, valid_columns)
        
        # 填充结果
        result.center_x = lane_center
        result.center_y = self.image_height // 2
        result.left_bound = left_bound
        result.right_bound = right_bound
        result.detected = True
        
        return result

    def detect_edges_with_opencv(self, binary_image):
        """使用OpenCV的Canny边缘检测算法"""
        height = len(binary_image)
        width = len(binary_image[0]) if height > 0 else 0
        
        # 转换为numpy数组
        binary_np = np.array(binary_image, dtype=np.uint8)
        
        # 应用Canny边缘检测
        edges = cv2.Canny(binary_np, 50, 150)
        return edges

    def extract_lane_edges(self, edges):
        """从边缘图像中提取左右车道边缘"""
        height, width = edges.shape[:2]
        edge_counts = np.zeros(width, dtype=int)
        
        # 计算每列的边缘像素数量
        for x in range(width):
            edge_counts[x] = np.sum(edges[:, x] == 255)
        
        mid_x = width // 2
        left_max = 0
        right_max = 0
        left_peak = 0
        right_peak = 0
        
        # 找到左半部分边缘最多的列
        for x in range(mid_x):
            if edge_counts[x] > left_max:
                left_max = edge_counts[x]
                left_peak = x
        
        # 找到右半部分边缘最多的列
        for x in range(mid_x, width):
            if edge_counts[x] > right_max:
                right_max = edge_counts[x]
                right_peak = x
        
        # 收集左边缘像素
        left_edges = []
        if left_max > 0:
            for y in range(height):
                if edges[y, left_peak] == 255:
                    left_edges.append(left_peak)
        
        # 收集右边缘像素
        right_edges = []
        if right_max > 0:
            for y in range(height):
                if edges[y, right_peak] == 255:
                    right_edges.append(right_peak)
        
        return left_edges, right_edges

    def calculate_edge_based_center(self, left_edges, right_edges, valid_columns):
        """基于边缘计算车道中心"""
        if not valid_columns:
            return 0
        
        if left_edges and right_edges:
            left_avg = sum(left_edges) // len(left_edges)
            right_avg = sum(right_edges) // len(right_edges)
            return (left_avg + right_avg) // 2
        
        return (valid_columns[0] + valid_columns[-1]) // 2


def calculate_lane_center(image_data, width, height):
    """
    Python接口函数，计算车道中心
    :param image_data: 一维数组形式的二值化图像数据
    :param width: 图像宽度
    :param height: 图像高度
    :return: (center_x, center_y, success)，success为True表示成功
    """
    # 检查输入有效性
    if image_data is None or width <= 0 or height <= 0:
        return (-1, -1, False)
    
    try:
        # 转换为二维列表
        binary_image = []
        for y in range(height):
            row = image_data[y * width : (y + 1) * width]
            binary_image.append(row)
        
        # 计算车道中心
        calculator = LaneCalculator(width, height)
        result = calculator.calculate_lane_center(binary_image)
        
        if result.detected:
            return (result.center_x, result.center_y, True)
        else:
            return (-1, -1, False)
    
    except Exception as e:
        print(f"计算过程出错: {e}")
        return (-1, -1, False)