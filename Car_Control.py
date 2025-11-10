import smbus
import math

#基于I2C通信协议的小车控制
class Car():
    
    def get_i2c_device(self,address,i2c_bus):
        self._addr=address
        if i2c_bus is None:
            return smbus.SMBus(1)
        else:
            return smbus.SMBus(i2c_bus)
        
    def __init__(self):
        self._device=self.get_i2c_device(0x16,1)
        self.Ctrl_Servo(1,135,2,90)
    
    def write_u8(self,reg,data):
        try:
            self._device.write_byte_data(self._addr,reg,data)
        except:
            print('write_u8 I2C error')
    
    def write_reg(self,reg):
        try:
            self._device.write_byte_data(self._addr,reg)
        except:
            print('write_u8 I2C error')
    
    def write_array(self,reg,data):
        try:
            self._device.write_i2c_block_data(self._addr,reg,data)
        except:
            print('write_array I2C error')
    
    def Ctrl_Car(self,L_dir,speed1,R_dir,speed2):
        try:
            reg=0x01
            data=[L_dir,speed1,R_dir,speed2]
            self.write_array(reg,data)
        except:
            print('Ctrl_Car I2C error')
    
    def Dir_Car(self,speed1,speed2):
        try:
            if speed1<0:
                L_dir=0
            else:
                L_dir=1
            if speed2<0:
                R_dir=0
            else:
                R_dir=1
            self.Ctrl_Car(L_dir,int(math.fabs(speed1)),R_dir,int(math.fabs(speed2)))
        except:
            print('Dir_Car I2C error')
        
    def Car_Run(self,speed1,speed2):
        try:
            self.Ctrl_Car(1,speed1,1,speed2)
        except:
            print('Car_Run I2C error')
        
    def Car_Stop(self):
        try:
            reg=0x02
            self.write_u8(reg,0x00)
        except:
            print('Car_Stop I2C error')
                
    def Car_Back(self,speed1,speed2):
        try:
            self.Ctrl_Car(0,speed1,0,speed2)
        except:
            print('Car_Back I2C error')
                
    def Car_Left(self,speed1,speed2):
        self.Ctrl_Car(0,speed1,1,speed2)
            
    def Car_Right(self,speed1,speed2):
        self.Ctrl_Car(1,speed1,0,speed2)

    def Car_Left_Spin(self,speed1,speed2):
        self.Ctrl_Car(0,speed1,1,speed2)
            
    def Car_Right_Spin(self,speed1,speed2):
        self.Ctrl_Car(1,speed1,0,speed2)
        
    def Ctrl_Servo(self,id1,angle1,id2,angle2):
        reg=0x03
        self.write_array(reg,[id1,angle1])
        self.write_array(reg,[id2,angle2])
