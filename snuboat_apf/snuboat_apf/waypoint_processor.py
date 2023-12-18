import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

from std_msgs.msg import Float32
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

import math
import numpy as np



waypoint_sequence = [ 'waypoint_1', 'waypoint_2', 'waypoint_3' ]
waypoint_data = {

    'waypoint_1': {
        'x': 0.3,
        'y': 0.0,
        #'lon' : 128.57883167,
        #'lat' : 35.06964348,
        'time': 0
    },

    'waypoint_2': {
        'x': 0.0,
        'y': 0.3,
        #'lon' : 128.57892811,
        #'lat' : 35.06937904,
        'time': 0
    },

    'waypoint_3': {
        'x': 3.0,
        'y': 0.0,
        #'lon' : 128.57892811,
        #'lat' : 35.06937904,
        'time': 0
    },



}

timer_count = 0
waypoint_count = 0
arrival_radius = 0.2

x_data = Float32()
y_data = Float32()
arrival_radius_data = Float32()

pos=np.array([0.0,0.0]) # x, y
pos_imported = 0
longitude=0.0
latitude=0.0



class WaypointPublisher(Node):

    def __init__(self):
        super().__init__('waypoint_publisher')
        
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        
        
        self.publisher_x = self.create_publisher(Float32, '/destination/x', 10)
        self.publisher_y = self.create_publisher(Float32, '/destination/y', 10)
        self.publisher_arrival_radius = self.create_publisher(Float32, '/destination/arrival_radius', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) 
            
                   
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



    def latitude_callback(self, msg):
        global latitude

        latitude = float(msg.data)
        pos[1] = (latitude*10000)%1000

    def longitude_callback(self, msg):
        global longitude

        longitude = float(msg.data)
        pos[0] = (longitude*10000)%1000





    def timer_callback(self):
        
        global waypoint_count
        global timer_count
        global arrival_radius
        
        current_waypoint = waypoint_sequence[waypoint_count]
        
        x_data.data = waypoint_data[current_waypoint]['x']
        y_data.data = waypoint_data[current_waypoint]['y']
        #x_data.data = (waypoint_data[current_waypoint]['lon']*10000)%1000
        #y_data.data = (waypoint_data[current_waypoint]['lat']*10000)%1000
        arrival_radius_data.data = arrival_radius
        
        self.publisher_x.publish(x_data)
        self.publisher_y.publish(y_data)
        self.publisher_arrival_radius.publish(arrival_radius_data)
        
        x=(pos[0] - waypoint_data[current_waypoint]['x'])
        y=(pos[1] - waypoint_data[current_waypoint]['y'])
        #x=(pos[0] - (waypoint_data[current_waypoint]['lon']*10000)%1000 )
        #y=(pos[1] - (waypoint_data[current_waypoint]['lat']*10000)%1000 )

        distance = math.sqrt(x**2 + y**2)
        
        
        
        if (distance <= arrival_radius):
            timer_count+=1
        else:
            timer_count=0
            
            
        if (timer_count > 10 * waypoint_data[current_waypoint]['time'] and waypoint_count < len(waypoint_data)-1 ):
            waypoint_count+=1
            timer_count=0
        
        
        self.get_logger().info('waypoint = %d , timer = %d , distance = %f' % (waypoint_count, timer_count, distance ) )




    def pose_callback(self, pose):
        #global pos_imported
        #pos_imported=1
        pos[0] = pose.pose.pose.position.x
        pos[1] = pose.pose.pose.position.y



def main(args=None):

    rclpy.init(args=args)

    waypoint_publisher = WaypointPublisher()

    rclpy.spin(waypoint_publisher)

    waypoint_publisher.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()

