import control.Car_Control as Car_Control
import control.PID as PID
import time

global Z_axis_pid

#PID赋值
Z_axis_pid = PID.PositionalPID(0.6, 0, 1) 

#创建小车控制的对象
car=Car_Control.Car()

def PID_Turn(offsets):
    global Z_axis_pid
    
    #偏差值计算
    #offsets=159-int((left_x+right_x)*0.5)
    
    #转向角PID调节
    Z_axis_pid.SystemOutput=offsets
    Z_axis_pid.SetStepSignal(0)
    Z_axis_pid.SetInertiaTime(0.4,0.1)
    
    #定义转向最大幅度
    if Z_axis_pid.SystemOutput>60:
        Z_axis_pid.SystemOutput=60
    elif Z_axis_pid.SystemOutput<-60:
        Z_axis_pid.SystemOutput=-60
    
    #右转判定
    if offsets>3 and offsets<500:
        if offsets>120:
            car.Dir_Car(-70,60)
            
        else:
            car.Dir_Car(60-int(Z_axis_pid.SystemOutput),60+int(Z_axis_pid.SystemOutput))
            print('turn right')
        time.sleep(0.001)
    
    #左转判定      
    elif offsets<-3 and offsets>-500:
        if offsets<-120:
            car.Dir_Car(60,-70)

        else:
            car.Dir_Car(60-int(Z_axis_pid.SystemOutput),60+int(Z_axis_pid.SystemOutput))
            print('turn left')
        time.sleep(0.001)
        
    #错误返回值判定   
    elif offsets<-500 or offsets>500:
        print('find no lane')
        car.Car_Stop()
    
    #直行判定    
    else:
        #print('run stright')
        car.Car_Run(50,50)    
