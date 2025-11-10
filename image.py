import cv2
import numpy as np

def inverse_perspective(image):
    """
    Applies inverse perspective mapping to the input image to obtain a bird's-eye view.

    Parameters:
    image (numpy.ndarray): The input image.

    Returns:
    numpy.ndarray: The transformed bird's-eye view image.
    """
    # Define source and destination points for perspective transformation
    srcPt1 = (120, 240)
    srcPt2 = (210, 240)
    srcPt3 = (190, 0)
    srcPt4 = (140, 0)

    dstPts1 = (120, 240)
    dstPts2 = (210, 240)
    dstPts3 = (210, 0)
    dstPts4 = (120, 0)

    srcPts = np.float32([srcPt1, srcPt2, srcPt3, srcPt4])
    dstPts = np.float32([dstPts1, dstPts2, dstPts3, dstPts4])
    
    # Compute the perspective transformation matrix
    M = cv2.getPerspectiveTransform(srcPts, dstPts)

    # Get the dimensions of the input image
    img_size = (image.shape[1], image.shape[0])
    
    # Apply the perspective transformation
    birdseye_view = cv2.warpPerspective(image, M, img_size, flags=cv2.INTER_LINEAR)
    
    return birdseye_view
    
#图像预处理
def preprocess_image(image,threshold):
    
    gray=cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)
    blurred=cv2.GaussianBlur(gray,(5,5),0)
    _,binary=cv2.threshold(blurred,threshold,255,cv2.THRESH_BINARY_INV)
    
    return binary
    
 #获取感兴趣区域   
def get_roi(image):
    
    roi = image[144:240,0:320]
    
    return roi
