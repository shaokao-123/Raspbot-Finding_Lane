import Car_Control
import PID
import time


global Z_axis_pid
Z_axis_pid = PID.PositionalPID(0.6, 0, 1) 
car=Car_Control.Car()
def PID_Turn(center_x,camera_width):
    global Z_axis_pid
    sum1=0
    offsets=camera_width*0.5-center_x
    #转向角PID调节
    Z_axis_pid.SystemOutput=offsets
    Z_axis_pid.SetStepSignal(0)
    Z_axis_pid.SetInertiaTime(0.4,0.1)
    
    if Z_axis_pid.SystemOutput>60:#调节最大幅度
        Z_axis_pid.SystemOutput=60
    elif Z_axis_pid.SystemOutput<-60:
        Z_axis_pid.SystemOutput=-60
    

    
    if offsets>3 and offsets<500:
        sum1+=1
        if offsets>120:
            car.Dir_Car(-70,60)
            
        else:
            car.Dir_Car(60-int(Z_axis_pid.SystemOutput),60+int(Z_axis_pid.SystemOutput))
        time.sleep(0.001)
            
    elif offsets<-3 and offsets>-500:
        if offsets<-120:
            car.Dir_Car(60,-70)

        else:
            car.Dir_Car(60-int(Z_axis_pid.SystemOutput),60+int(Z_axis_pid.SystemOutput))
        time.sleep(0.001)
        
    elif offsets<-500 or offsets>500:
        car.Car_Stop()
        
    else:
        car.Car_Run(50,50)    
