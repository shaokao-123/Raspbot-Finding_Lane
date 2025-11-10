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
    
