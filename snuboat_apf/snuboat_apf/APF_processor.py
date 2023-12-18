import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

import math
import numpy as np

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
#from microstrain_inertial_msgs.msg import FilterHeading




#current_heading_angle = 0.0



apf = np.array([0.0,0.0]) # x, y

pos = np.array([0.0,0.0,0.0]) # x, y, z

quat = np.array([0.0,0.0,0.0,0.0]) # x, y, z, w

destination = np.array([0.0,0.0]) # x, y

data = PoseStamped()


class APFProcessor(Node):

    def __init__(self):
    
        super().__init__('apf_processor')
        
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile) 
            
                   
        self.subscription = self.create_subscription(
            Odometry,
            '/odom_rf2o',
            self.pose_callback,
            qos_profile) 

  


        self.subscription = self.create_subscription(
            String,
            '/gps/lat',
            self.latitude_callback,
            qos_profile)      


        self.subscription = self.create_subscription(
            String,
            '/gps/lon',
            self.longitude_callback,
            qos_profile)


        '''
        self.subscription = self.create_subscription(
            FilterHeading,
            '/nav/heading',
            self.heading_callback,
            qos_profile)
        ''' 
                   
        self.subscription = self.create_subscription(
            Float32,
            '/destination/x',
            self.destination_x_callback,
            qos_profile) 
            
                   
        self.subscription = self.create_subscription(
            Float32,
            '/destination/y',
            self.destination_y_callback,
            qos_profile)
            
            
        self.publisher_ = self.create_publisher(PoseStamped, 'desired_heading', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        





#########################################################
####  import & process 'lidar scan data'


    def lidar_callback(self, scan):
        
        lidar_array = np.array(scan.ranges)
        
        #lidar_array = np.reciprocal(lidar_array, out=np.zeros_like(lidar_array), where=lidar_array!=0) / len(scan.ranges)
        
        
        #lidar_array = lidar_array*5  #covariance
        
        
        x=0.0
        y=0.0
        
        
        for i in range(0,len(lidar_array)-1):
            if (lidar_array[i]<2.5 and lidar_array[i]>0):
                angle= ( (i-(len(lidar_array)/4)) / len(lidar_array) ) * 2 * math.pi
                #print(angle)
                #angle= (i*2* math.pi/len(lidar_array))
                x -= ((2.5-lidar_array[i])*math.sin(angle))
                y += ((2.5-lidar_array[i])*math.cos(angle))       
        
        #print(lidar_array[500])
        apf[0] = 5*x/math.sqrt(x*x+y*y)
        apf[1] = 5*y/math.sqrt(x*x+y*y)
        
        
        #self.get_logger().info('I heard: "%f,  %f,  %f"' % (  len(lidar_array), apf_x, apf_y  )  )

        #self.get_logger().info('I heard: "%f,  %f,  %f"' % (  pos_x, pos_y, 0.0  )  )

#########################################################
####  import 'pose data' from SLAM


    def pose_callback(self, pose):
        
        pos[0] = pose.pose.pose.position.x
        pos[1] = pose.pose.pose.position.y
        pos[2] = pose.pose.pose.position.z
        
        quat[0] = pose.pose.pose.orientation.x
        quat[1] = pose.pose.pose.orientation.y
        quat[2] = pose.pose.pose.orientation.z
        quat[3] = pose.pose.pose.orientation.w
        
        data.header = pose.header
        
        '''
        current_heading_angle = np.arctan2(2*(quat_w*quat_z + quat_x*quat_y), 1 - 2*(quat_y**2 + quat_z**2))
        current_heading_angle = np.degrees(yaw)
        self.get_logger().info('I heard: "%f,  %f,  %f"' % (  yaw, pos_x, pos_y  )  )
        '''
        #self.get_logger().info('I heard:')



    def heading_callback(self, msg):
        global current_yaw

        current_yaw = msg.heading_deg



    def latitude_callback(self, msg):
        global latitude

        latitude = float(msg.data)
        pos[1] = (latitude*10000)%1000

    def longitude_callback(self, msg):
        global longitude

        longitude = float(msg.data)
        pos[0] = (longitude*10000)%1000
        


#########################################################
####  import 'destination info'


    def destination_x_callback(self, msg):
        destination[0] = msg.data
        
        #self.get_logger().info('dest_x  %f' % destination_x)

    def destination_y_callback(self, msg):
        destination[1] = msg.data
        
        #self.get_logger().info('dest_y  %f' % destination_y)



#########################################################
####  process and publish 


    def timer_callback(self):
        
        covariance = 0.7
        
        # 목표지점이랑 현재위치 빼서 목표방향구하기
        desired_x = destination[0] - pos[0]
        desired_y = -destination[1] + pos[1]
        
        # 방향 벡터 크기 1로 만들기 
        magnitude = np.sqrt(desired_x**2 + desired_y**2)
        if(magnitude!=0):
            desired_x = desired_x/magnitude
            desired_y = desired_y/magnitude
        
        # 벡터에 상수 곱하기
        desired_x *= covariance
        desired_y *= covariance
        
        # 방향벡터에 apf 계산결과 더하기
        desired_x += apf[0]
        desired_y += apf[1]
        
        # 방향 벡터 크기 1로 만들기 
        magnitude = np.sqrt(desired_x**2 + desired_y**2)
        if(magnitude!=0):
            desired_x = desired_x/magnitude
            desired_y = desired_y/magnitude
        
        
        
        
        
        data.pose.position.x = pos[0]
        data.pose.position.y = pos[1]
        data.pose.position.z = pos[2]
        data.pose.orientation.x = desired_x
        data.pose.orientation.y = desired_y
        data.pose.orientation.z = 0.0
        data.pose.orientation.w = 0.0	
        
        self.publisher_.publish(data)
        self.get_logger().info('Publishing: %f , %f' % (desired_x, desired_y) )



#########################################################




def main(args=None):

    rclpy.init(args=args)

    apf_processor = APFProcessor()

    rclpy.spin(apf_processor)

    apf_processor.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
