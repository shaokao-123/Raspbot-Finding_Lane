class IncrementalPID:
    def __init__(self, P, I, D):
        self.Kp = P
        self.Ki = I
        self.Kd = D
 
        self.PIDOutput = 0.0             #PID控制器输出
        self.SystemOutput = 0.0          #系统输出值
        self.LastSystemOutput = 0.0      #上次系统输出值
 
        self.Error = 0.0                 #输出值与输入值的偏差
        self.LastError = 0.0
        self.LastLastError = 0.0
 
    #设置PID控制器参数
    def SetStepSignal(self,StepSignal):
        self.Error = StepSignal - self.SystemOutput
        #增量值
        IncrementValue = self.Kp * (self.Error - self.LastError) +\
        self.Ki * self.Error +\
        self.Kd * (self.Error - 2 * self.LastError + self.LastLastError)

        self.PIDOutput += IncrementValue
        self.LastLastError = self.LastError
        self.LastError = self.Error

    #设置一阶惯性环节系统  其中InertiaTime为惯性时间常数（简单认为某一变量达到其预设目标值固定百分比用的时间），SampleTime为采样周期
    def SetInertiaTime(self,InertiaTime,SampleTime):
        self.SystemOutput = (InertiaTime * self.LastSystemOutput + \
            SampleTime * self.PIDOutput) / (SampleTime + InertiaTime)

        self.LastSystemOutput = self.SystemOutput
        
class PositionalPID:
    def __init__(self, P, I, D):
        self.Kp = P
        self.Ki = I
        self.Kd = D
 
        self.SystemOutput = 0.0   #系统输出
        self.LastSystemOutput = 0.0  #系统上次输出
        self.PidOutput = 0.0  #控制器输出
        self.PIDErrADD = 0.0  #误差累加
        self.LastError = 0.0  #上次误差
    
    #设置PID控制器参数
    def SetStepSignal(self,StepSignal):
        Error = StepSignal - self.SystemOutput
        self.PidOutput = self.Kp * Error + self.Ki * self.PIDErrADD + self.Kd * (Error - self.LastError)
        self.PIDErrADD += Error
        if self.PIDErrADD > 2000:
            self.PIDErrADD = 2000
        if self.PIDErrADD < -2500:
            self.PIDErrADD = -2500
        self.LastError = Error
        

    #设置一阶惯性环节系统,同上
    def SetInertiaTime(self, InertiaTime,SampleTime):
           self.SystemOutput = (InertiaTime * self.LastSystemOutput + \
           SampleTime * self.PidOutput) / (SampleTime + InertiaTime)

           self.LastSystemOutput = self.SystemOutput
       