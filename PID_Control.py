import Car_Control
import PID
import time


global Z_axis_pid
Z_axis_pid = PID.PositionalPID(0.5, 0, 1) 
global prev_left
prev_left = 0
global prev_right
prev_right = 0
car=Car_Control.Car()
def PID_Turn(left_x,right_x,offsets,left_lane_sum,right_lane_sum):
    global prev_left, prev_right
    global Z_axis_pid
    
    #转向角PID调节
    Z_axis_pid.SystemOutput=offsets
    Z_axis_pid.SetStepSignal(0)
    Z_axis_pid.SetInertiaTime(0.3,0.1)
    
    if Z_axis_pid.SystemOutput>60:#调节最大幅度
        Z_axis_pid.SystemOutput=60
    elif Z_axis_pid.SystemOutput<60:
        Z_axis_pid.SystemOutput=60
    

    
    if left_x==0 and right_x==319:
        if prev_left>prev_right:
            car.Dir_Car(-70,60)
        elif prev_left<prev_right:
            car.Dir_Car(60,-70)
        
        prev_left=0
        prev_right=0
        
    else:
        if offsets>3:
            if offsets>120:
                car.Dir_Car(-70,60)
                prev_left=0
                prev_right=0
            else:
                car.Dir_Car(45+int(Z_axis_pid.SystemOutput),45-int(Z_axis_pid.SystemOutput))
            time.sleep(0.001)
            
        elif offsets>3:
            if offsets<-120:
                car.Dir_Car(60,-70)
                prev_left=0
                prev_right=0
            else:
                car.Dir_Car(45+int(Z_axis_pid.SystemOutput),45-int(Z_axis_pid.SystemOutput))
            time.sleep(0.001)
        
        else:
            car.Car_Run(25,25)    
    
    if left_lane_sum != right_lane_sum:
        if left_lane_sum < right_lane_sum:
            prev_left = prev_left + 1
        elif right_lane_sum < left_lane_sum:
            prev_right = prev_right + 1
        