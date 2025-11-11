import PID
import Car_Control
import time

#PID赋值
sport = PID.PositionalPID(0.6,0,1)

#创建小车对象
car = Car_Control.Car() 
def PID_Turn(offsets):
    
    #offsets = 159 - center_x
    sport.SystemOutput = offsets
    sport.SetStepSignal(0)#这一行运行时，有error(误差) = 0 - sport.SystemOutput，又error与下一行sport.SystemOutput的再赋值有关，当offsets大于0时，条件语句中的sport.SystemOutput＜0
    sport.SetInertiaTime(0.1,0.01)


    if sport.SystemOutput > 30:
        sport.SystemOutput = 30
    elif sport.SystemOutput < -30:
        sport.SystemOutput = -30
    #如果不限制最大系统输出?


    if offsets > 15:#黑线在中线左边，小车左转。
        
        if offsets > 140:
            car.Dir_Car(-70,70)
        #当去除偏差较大时小车大幅转弯时，能否消除当时突然停止往相反方向转弯的影响
        
        else:
            car.Dir_Car(60+int(sport.SystemOutput),60-int(sport.SystemOutput))
            print('turn left')
        time.sleep(0.001)

    elif offsets < -15 and offsets > -161:
        '''
        if offsets < -140:
            car.Dir_Car(70,-70)
        '''
        #else:
        car.Dir_Car(60+int(sport.SystemOutput),60-int(sport.SystemOutput))
        print('turn right')
        time.sleep(0.001)
        
    elif offsets < -500:
        car.Car_Stop()

    else:
        car.Car_Run(60,60)
