import rclpy
import serial
import threading
from rclpy.node import Node
from std_msgs.msg import Int32, Int64, String
import numpy as np


imu_serial=serial.Serial(port='/dev/ttyIMU', baudrate=115200)
arduino_serial=serial.Serial(port='/dev/ttyARDUINO', baudrate=115200)


autopilot_mode=0 # 0=straight, 1=zigzag
Kp=10.0
Kd=10.0
move_pwm=1800
zigzag_seconds=3.0


system_counter=0  #젯슨은 약해요..1초에 10번만 계산해줄거
yaw=0.0
previous_relative_yaw=0.0
yaw_measuring_array = np.array([])
reference_yaw=0.0

class Autopilot_3(Node):


    def __init__(self):
        super().__init__('autopilot_3')
        
        timer_period =0.01
        self.timer = self.create_timer(timer_period,self.timer_callback)

    def timer_callback(self): #1초에 100번씩 실행됨
    
        global system_counter

        self.imu_reader()

        if system_counter%10==0:

            mode=self.lifecycle_process() # 0=measure, 1=set, 2=str, 3=zigzag1, 4=zigzag2

            pwm_data=self.dynamics_process(mode)

            self.arduino_sender(pwm_data)
            
            print('yaw: '+str(reference_yaw)+' pwm: '+str(pwm_data)+' mode: '+str(mode)+' ' +str(system_counter/10))
            

        system_counter+=1

        
   
    #################################################

    def lifecycle_process(self):
        global autopilot_mode
        global yaw_measuring_array
        global yaw
        global zigzag_seconds
        global reference_yaw

        if system_counter<500:
            yaw_measuring_array=np.append(yaw_measuring_array,[yaw],axis=0)
            return 0

        elif system_counter==500:
            reference_yaw=np.mean(yaw_measuring_array)
            return 1

        elif system_counter>500 and autopilot_mode==0:
            return 2

        elif system_counter>500 and autopilot_mode==1:
            if int(system_counter/(zigzag_seconds*100))%2==0:
                return 3
            elif int(system_counter/(zigzag_seconds*100))%2==1:
                return 4


    #################################################

    def dynamics_process(self, mode):
        global move_pwm
        global Kp
        global Kd
        global yaw
        global reference_yaw
        global previous_relative_yaw
        

        if mode==2:

            relative_yaw= ((yaw - reference_yaw) + 180) % 360 - 180

        elif mode==3:

            relative_yaw= ((yaw - (reference_yaw+20)) + 180) % 360 - 180

        elif mode==4:

            relative_yaw= ((yaw - (reference_yaw-20)) + 180) % 360 - 180
        
        else:
            relative_yaw=0.0


         


        left_pwm=move_pwm - Kp*relative_yaw + Kd*(relative_yaw-previous_relative_yaw)
        right_pwm=move_pwm + Kp*relative_yaw - Kd*(relative_yaw-previous_relative_yaw)



        if mode==0 or mode==1:
            left_pwm=1500
            right_pwm=1500

        previous_relative_yaw=relative_yaw
        
        #print(left_pwm)
        #print(right_pwm)
        if left_pwm>1900:
            left_pwm=1900
        if left_pwm<1100:
            left_pwm=1100
        if right_pwm>1900:
            right_pwm=1900
        if right_pwm<1100:
            right_pwm=1100
         
        return int(left_pwm)*10000 + int(right_pwm)



    #################################################

    #imu 신호 parcing
    def imu_reader(self):
        global yaw
        

        if system_counter%10==0:    #젯슨은 약해요...1초에 10번만 읽을거
            
            raw_data = (imu_serial.readline()).decode('utf-8')
            
            if raw_data[0] =='*':

                data = (raw_data[1:-2]).split(',')
                #print(data[2])
                yaw=float(data[2])


        else:         # 다른때는 그냥 데이터 파쇄ㅋㅋ
            imu_serial.readline()
                   
        
        

    #################################################


    #################################################

    #arduino에 신호 보냄
    def arduino_sender(self, pwm):

        arduino_serial.write(f'{pwm}\n'.encode())

    #################################################




        

def main(args=None):

    rclpy.init(args=args)

    autopilot_3 = Autopilot_3()

    rclpy.spin(autopilot_3)


    autopilot_3.destroy_node()
    
    
    arduino_serial.write('15001500'.encode())

    rclpy.shutdown()



if __name__ == '__main__':
    main()
