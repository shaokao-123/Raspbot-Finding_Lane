import image
import cv2
#from ctype import detect_lane_center
import numpy as np
import LCL2

def detect_lane_center(roi):
    lcl_detector = LCL2.LaneCalculator(width=roi.shape[1], height=roi.shape[0])
    center_x, center_y = lcl_detector.calculate_lane_center(roi).center_x, lcl_detector.calculate_lane_center(roi).center_y
    return center_x, center_y

with open("./results/warped_0007.jpg", "rb") as f:
    img_data = f.read()
    img = cv2.imdecode(np.frombuffer(img_data, np.uint8), cv2.IMREAD_COLOR)
    birdseye_view = image.inverse_perspective(img)
    binary = image.preprocess_image(birdseye_view)
    roi = image.get_roi(binary)
    result = detect_lane_center(roi)
    center_x, center_y = result[0], result[1]
    print(f"Detected lane center at: ({center_x}, {center_y})")
