# obstacle_avoidance using enu coordinate from gnss_converter 

# KABOAT
import rclpy
import os
import yaml
from rclpy.node import Node
from geometry_msgs.msg import Pose,Point, TwistWithCovarianceStamped, PoseStamped
from microstrain_inertial_msgs.msg import FilterHeading
from std_msgs.msg import Bool, Int8, Int32, Float32, Float64, String, Float64MultiArray, Int32MultiArray, Int8MultiArray
from nav_msgs.msg import Odometry
import numpy as np
import math
import tf2_ros
import pymap3d as pm

class Obstacle_Avoidance(Node):
    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time} [{name}] [{severity}] {message}'
    def __init__(self):
        super().__init__("obstacle_avoidance")

        # Dimension
        # Big Ship
        self.L = 0.9
        self.B = 0.3
        # Small Ship 
        # self.L = 0.7
        # self.B = 0.23

        self.dt = 0.1
        self.cur_wp_idx = 0

        # Small Tank
        # self.safe_radius = 1.5 
        # self.ref_safe_radius = 1.5
        # Long Tank
        self.safe_radius = 1.5
        self.ref_safe_radius = 1.5

        # self.safe_radius = 1.7
        # self.ref_safe_radius = 1.75
        self.goal_tol = 1.5
        self.dock_goal_tol = 0.5
        self.ref_goal_tol = 1.5
        self.wp_stay_time = 30
        self.inf_length = self.B * 1.0
        self.ref_inf_length = self.B * 1.0

        
        # wp variablej
        # self.left_top = [-6.735951374889645, -0.1664803763013909]
        # self.right_top = [-7.193387138591193, 3.9844984851497918]
        # self.left_bottom = [0.0, 0.0]
        # self.right_bottom = [-0.645900644634858, 4.361856980183751]

        # gnss
        self.right_top_gnss = [35.06935917, 128.57892450, 0.0]
        self.left_top_gnss = [35.06938700, 128.57902350, 0.0]
        self.left_bottom_gnss = [35.06968750, 128.57890267, 0.0]
        self.right_bottom_gnss = [35.06965917, 128.57880283, 0.0]
        self.origin  = self.left_bottom_gnss

        self.right_top = [0.0, 0.0, 0.0]
        self.right_bottom = [0.0, 0.0, 0.0]
        self.left_top = [0.0, 0.0, 0.0]
        self.left_bottom = [0.0, 0.0, 0.0]

        self.right_top[0], self.right_top[1], self.right_top[2] = \
            pm.geodetic2enu(self.right_top_gnss[0], self.right_top_gnss[1], self.right_top_gnss[2], self.origin[0], self.origin[1], self.origin[2])
        self.left_top[0], self.left_top[1], self.left_top[2] = \
            pm.geodetic2enu(self.left_top_gnss[0], self.left_top_gnss[1], self.left_top_gnss[2], self.origin[0], self.origin[1], self.origin[2])
        self.left_bottom[0], self.left_bottom[1], self.left_bottom[2] = \
            pm.geodetic2enu(self.left_bottom_gnss[0], self.left_bottom_gnss[1], self.left_bottom_gnss[2], self.origin[0], self.origin[1], self.origin[2])
        self.right_bottom[0], self.right_bottom[1], self.right_bottom[2] = \
            pm.geodetic2enu(self.right_bottom_gnss[0], self.right_bottom_gnss[1], self.right_bottom_gnss[2], self.origin[0], self.origin[1], self.origin[2])

        # Data
        # data_x = np.array([-0., 11.0210301, -9.10648011, 1.9911377]) # left bottom, left top, right bottom, right top
        # data_y = np.array([0., -33.33801876, -3.14297808, -36.42553678])

        # data_x1 = np.array([ 2.89412663, -6.29080575])
        # data_y1 = np.array([-34.26993648,  -4.77049741])

        # self.odom_wp_x_set = np.array([self.right_top[0], self.left_top[0], self.left_bottom[0], self.right_bottom[0]])
        # self.odom_wp_y_set = np.array([self.right_top[1], self.left_top[1], self.left_bottom[1], self.right_bottom[1]])

        # odom_wp_x_0, odom_wp_y_0, odom_wp_z_0 = pm.geodetic2enu(35.0693786, 128.57893439999998, 0.0, self.origin[0], self.origin[1], self.origin[2]) # close to right top
        # odom_wp_x_1, odom_wp_y_1, odom_wp_z_1 = pm.geodetic2enu(35.069644499999995, 128.5788337, 0.0, self.origin[0], self.origin[1], self.origin[2]) # close top right bottom

        # self.odom_wp_x_set = np.array([odom_wp_x_0, odom_wp_x_1, odom_wp_x_2, odom_wp_x_3])
        # self.odom_wp_y_set = np.array([odom_wp_y_0, odom_wp_y_1, odom_wp_y_2, odom_wp_y_3])
        
        data_x1 = 2.89412663
        data_y1 = -34.26993648

        data_x_dock_start = 8.359257006082448 
        data_y_dock_start = -33.335876593669234

        data_x_dock1 = [6.1311213530135005, 7.725739602627772,  9.320357852242045]
        data_y_dock1 = [-31.969540121427183, -31.423881781446475, -30.87822344146576]

        data_x_dock2 = [5.406215202482971, 7.001719712965285, 8.597224223447194]
        data_y_dock2 = [-29.78149762680809, -29.238744853191896, -28.686656970607498]

        data_x_dock3 = [3.2314967508919965, 4.829660043977821, 6.427823337063646] 
        data_y_dock3 = [-23.23155950858245, -22.68333406842816, -22.135108628273866]
        
        self.dock_wp_idx = 2
        
        self.dock_wp_x_set = np.array([[6.1311213530135005, 7.725739602627772,  9.320357852242045], \
            [5.406215202482971, 7.001719712965285, 8.597224223447194], \
                [3.2314967508919965, 4.829660043977821, 6.427823337063646]])
        
        self.dock_wp_y_set = np.array([[-31.969540121427183, -31.423881781446475, -30.87822344146576], \
            [-29.78149762680809, -29.238744853191896, -28.686656970607498], \
                [-23.23155950858245, -22.68333406842816, -22.135108628273866]])

        data_x2 = 3.8341326956918995
        data_y2 = -19.678770792078108

        data_x3 = -0.3289816698674061
        data_y3 = -7.114233454614271

        data_x_goal = -6.29080575
        data_y_goal = -4.77049741

        self.odom_wp_x_set = np.array([data_x1, data_x_dock_start, data_x_dock1[1], data_x_dock2[1], data_x_dock3[1], data_x2, data_x3, data_x_goal])
        self.odom_wp_y_set = np.array([data_y1, data_y_dock_start, data_y_dock1[1], data_y_dock2[1], data_y_dock3[1], data_y2, data_y3, data_y_goal])

        
        # 2023.08.06 waypoint number should be 7
        # self.odom_wp_x_set = np.array([self.a])

        # Rectangle CW-shape (Small Tank)
        # self.odom_wp_x_set = [0.0, 5.0, 5.0,0.0]
        # self.odom_wp_y_set = [4.0,4.0,0.0,0.0]

        # Rectangle X-shape (Small Tank)
        # self.odom_wp_x_set = [5.0, 5.0, 1.0, 1.0]
        # self.odom_wp_y_set = [6.0, 1.0, 6.0, 1.0]
        # self.odom_wp_x_set = [4.0, 4.0, 1.0, 1.0]
        # self.odom_wp_y_set = [5.0, 1.0, 5.0, 1.0]

        # self.odom_wp_x_set = [0.0, 5.0]
        # self.odom_wp_y_set = [0.0, 0.0]
        
        # Straight Line, Repeat, X-Direction (Long Tank)
        # self.odom_wp_x_set = [10.0,0.0, 10.0,0.0,10.0,0.0]
        # self.odom_wp_y_set = [0.0,0.0, 0.0,0.0,0.0,0.0]

        # Triangle (Long Tank)
        # self.odom_wp_x_set = [6.0, 6.0, 0.0, 6.0, 6.0, 0.0]
        # self.odom_wp_y_set = [3.0, -3.0, 0.0, 3.0, -3.0, 0.0]
        # self.odom_wp_x_set = [9.0, 9.0, 0.0, 9.0, 9.0, 0.0]
        # self.odom_wp_y_set = [3.0, -3.0, 0.0, 3.0, -3.0, 0.0]

        # Straight Line, Repeat, Y-Direction (Long Tank)
        # self.odom_wp_x_set = [3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 0.0]
        # self.odom_wp_y_set = [3.0, 0.0, -3.0, 3.0, 0.0, -3.0, 0.0]
        
        # Trajectory
        # 3 obs 
        # self.odom_wp_x_set = [6.5, 6.5, 6.5, 0.0, 6.5, 6.5,6.5,0.0]
        # self.odom_wp_y_set = [3.0, 0.0, -3.0, 0.0, 3.0, 0.0,-3.0,0.0]

        # 1 obs Str
        # self.odom_wp_x_set = [10.0, 0.0, 10.0, 0.0]
        # self.odom_wp_y_set = [0.0, 0.0, 0.0, 0.0]

        #subscriber

        #subscribe obstacle information
        self.obs_labels_sub = self.create_subscription(
            Int32MultiArray, "/obs/labels", self.obs_labels_callback, 1
        )
        self.obs_r_sub = self.create_subscription(
            Float64MultiArray, "/obs/r", self.obs_r_callback, 1
        )
        self.obs_phi_sub = self.create_subscription(
            Float64MultiArray, "/obs/phi", self.obs_phi_callback, 1
        )
        self.obs_x_sub = self.create_subscription(
            Float64MultiArray, "/obs/x", self.obs_x_callback, 1
        )
        self.obs_y_sub = self.create_subscription(
            Float64MultiArray, "/obs/y", self.obs_y_callback, 1
        )
        
        #subscribe from ekf filter
        # self.odom_sub = self.create_subscription(
        #     Odometry, "/odometry/filtered", self.odom_callback, 1
        # )
        # self.robot_pos_sub = self.create_subscription(
        #     Pose,"/robot_pose",self.robot_pose_callback, 1
        # )

        # subscribe from gnss_converter
        self.enu_pos_sub = self.create_subscription(
            Point, "/enu_pos", self.enu_pos_callback, 1
        )

        #subscribe from imu
        self.heading_sub = self.create_subscription(
            FilterHeading, "/nav/heading", self.heading_callback, 1
        )

        # self.enu_wp_x_set_sub = self.create_subscription(
        #     Float64MultiArray, "/enu_wp_set/x", self.enu_wp_x_set_callback, 1
        # )
        # self.enu_wp_y_set_sub = self.create_subscription(
        #     Float64MultiArray, "/enu_wp_set/y", self.enu_wp_y_set_callback, 1
        # )

        #subscribe from Shape_Detector
        self.shape_detect_sub = self.create_subscription(
            Int8MultiArray, "/detect_state", self.shape_detect_callback, 1
        ) # [Detect_state(0/1), Docking_position(-1/0/1)]

        #publisher
        self.des_heading_pub = self.create_publisher(Float64, "/des_heading", 1)
        self.des_spd_pub = self.create_publisher(Float64, "/des_spd", 1)
        self.cur_wp_idx_pub = self.create_publisher(Int8, "/wp_idx", 1)
        self.wp_set_x_pub = self.create_publisher(Float64MultiArray, "wp_set/x", 1)
        self.wp_set_y_pub = self.create_publisher(Float64MultiArray, "wp_set/y", 1)
        self.wp_check_pub = self.create_publisher(Bool,"/wp_check",1)
        self.wp_clear_pub = self.create_publisher(Bool,"/wp_clear",1)
        self.err_heading_pub = self.create_publisher(Float64,"/err_heading",1)
        self.ref_heading_pub = self.create_publisher(Float64,"/ref_heading",1)
        self.safe_heading_pub = self.create_publisher(Float64MultiArray,"/safe_heading",1)
        self.des_pub = self.create_timer(self.dt, self.pub_des)
        self.err_x_pub = self.create_publisher(Float64, "/err_x",1)
        self.err_y_pub = self.create_publisher(Float64,"/err_y",1)
        self.err_heading_next_pub = self.create_publisher(Float64, "/err_heading_next", 1)
        self.docking_state_pub = self.create_publisher(Bool, "/docking_state", 1)
        self.target_shape_pub = self.create_publisher(String,"/target_shape",1)

        # self.des_pub = self.create_timer(1.0, self.pub_des)
        # self.des_pub = self.create_timer(3.0, self.pub_des)
        # self.dock_pub = self.create_timer(3.0, self.pub_dock)

        self.obs_labels_received = False
        self.obs_r_received = False
        self.obs_phi_received = False
        self.obs_x_received = False
        self.obs_y_received = False

        # self.odom_received = False
        # self.robot_pose_received = False
        self.enu_pos_received = False

        self.heading_received = False
        self.spd_received = False
        self.obstacles_received = False
        # self.enu_wp_x_received = False
        # self.enu_wp_y_received = False

        self.detect_received = False

        # odom info
        self.odom_pos = []
        self.enu_pos = [0, 0]
        # self.odom_pos = np.zeros((10, 2))
        self.odom_orientation = []
        self.odom_twist = []
        self.odom_twist_ang = []
        self.ref_heading = 0.0
        self.wp_reach_check = False
        self.wp_time_cnt = 0
        
        self.wp_clear = False
        
        # Docking State
        self.docking_state = False
        self.detect_state = False
#############################################
        #Docking image
        self.target_shape = "green_circle"
#####################################################
        self.wp_state = False
        self.ref_spd = 0.8
        
        self.safe_heading = []
        self.heading_cost = []
        self.des_heading = np.zeros(10)  # why 10?
        self.heading = np.zeros(10)
        # self.des_spd = np.zeros(10)
        self.des_spd = np.ones(10)
        self.obs_labels = []
        self.err_heading = np.zeros(10)
        
        self.obs_r=[]
        self.obs_phi=[]
        self.obs_x=[]
        self.obs_y=[]
        # self.odom_wp_x_set=[5]
        # self.odom_wp_y_set=[7]
        #margin
        self.inflate_obs_phi = np.deg2rad(20)
        # self.inflate_obs_phi = np.deg2rad(0)

    def wait_for_topics(self):
        self.timer = self.create_timer(1.0, self.check_topic_status)
        # self.togodist()

    def check_topic_status(self):
        # if not self.odom_received:
        #     self.get_logger().info("No topic odom_received")
        if not self.enu_pos_received:
            self.get_logger().info("No topic enu_pos_received")

        if not self.heading_received:
            self.get_logger().info("No topic heading_received")

        if not self.obstacles_received:
            self.get_logger().info("No topic obstacles_received")

        # if not self.enu_wp_x_received:
        #     self.get_logger().info("No topic enu_wp_x_received")

        if (
            self.enu_pos_received
            and self.obs_labels_received
            and self.heading_received
            # and self.enu_wp_x_received
        ):
            self.get_logger().info("All topics received")
        else:
            self.get_logger().info("Waiting for topics to be published")
        # print("this is test")
        # print("cur_wp_idx", self.cur_wp_idx, \
        #       "To go:", self.print_dist, \
        #     "des_heading", np.rad2deg(self.des_heading[-1]), \
        #       "des_spd", self.des_spd[-1],\
        #       "wp_check",self.wp_reach_check)

    # def togodist(self):
    #     dist = np.linalg.norm([self.enu_pos[-1, 0] - self.odom_wp_x_set[self.cur_wp_idx], self.enu_pos[-1, 1] - self.odom_wp_y_set[self.cur_wp_idx]])
    #     self.get_logger().info('To go distance: ' + str(dist))



    def obs_labels_callback(self, msg):
        self.obs_labels_received = True
        # print(self.obs_labels_received)
        self.obs_labels = np.array(msg.data)
        # self.obs_labels = (self.obs_labels)
        # self.obs_labels = np.reshape(self.obs_labels, (1, -1))
        # self.obs_labels = self.obs_labels.flatten()

    def obs_r_callback(self, msg):
        self.obs_r_received = True
        self.obs_r = (np.array(msg.data))
        # self.obs_r = np.reshape(self.obs_r, (1, -1))
        # self.obs_labels = self.obs_labels.flatten()
        
    def obs_phi_callback(self, msg):
        self.obs_phi_received = True
        self.obs_phi = np.array(msg.data)
        # self.obs_phi = np.flip(self.obs_phi)
        # self.obs_phi = np.reshape(self.obs_phi, (1, -1))
        # self.obs_phi = self.obs_phi.flatten()

        # print("obs_phi")
        # print(self.obs_phi)

    def obs_x_callback(self, msg):
        self.obs_x_received = True
        self.obs_x = (np.array(msg.data))
        # self.obs_x = np.reshape(self.obs_x, (1, -1))
        # self.obs_x = self.obs_x.flatten()

    def obs_y_callback(self, msg):
        self.obs_y_received = True
        self.obs_y = (np.array(msg.data))
        # self.obs_y = np.reshape(self.obs_y, (1, -1))
        # self.obs_y = self.obs_y.flatten()
        
    # def robot_pose_callback(self,msg):
    #     self.robot_pose_received = True
    #     self.robot_pos = [msg.position.x,msg.position.y]
    #     self.robot_orientation = [msg.orientation.x,msg.orientation.y,
    #                               msg.orientation.z,msg.orientation.w]

    # def odom_callback(self, msg):
    #     self.odom_received = Trueradius
# tation.x,msg.pose.pose.orientation.y,
    #                              msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]
    #     self.odom_twist = [msg.twist.twist.linear.x,msg.twist.twist.linear.y,
    #                         msg.twist.twist.linear.z]
    #     self.odom_twist_ang = [msg.twist.twist.angular.x,msg.twist.twist.angular.y,
    #                            msg.twist.twist.angular.z]
        
    def enu_pos_callback(self, msg):
        self.enu_pos_received = True
        self.enu_pos = [msg.x, msg.y]

    def heading_callback(self, msg):
        self.heading_received = True
        # print(self.heading_received)
        self.heading = np.append(self.heading, msg.heading_rad)
        self.heading = self.heading[1:]

    # def enu_wp_x_set_callback(self, msg):
    #     self.enu_wp_x_received = True

    #     # who did that??
    #     # Surge young <<<<<<
    #     self.odom_wp_x_set = np.array(msg.data)
    #     # self.odom_wp_x_set = np.reshape(self.odom_wp_x_set, (1, -1))
    
    # def enu_wp_y_set_callback(self, msg):
    #     self.enu_wp_y_received = True
    #     self.odom_wp_y_set = np.array(msg.data)
    #     # self.odom_wp_y_set = np.reshape(self.odom_wp_y_set, (1, -1))

    def shape_detect_callback(self, msg):
        self.detect_received = True
        if self.docking_state == True and msg.data[0] == 1: 
            if msg.data[1] == -1:
                self.get_logger().info("Changing waypoint to docking point: Left")
                self.odom_wp_x_set[self.dock_wp_idx] = self.dock_wp_x_set[0, 0]
                self.odom_wp_y_set[self.dock_wp_idx] = self.dock_wp_y_set[0, 0]
                self.odom_wp_x_set[self.dock_wp_idx+1] = self.dock_wp_x_set[1, 0]
                self.odom_wp_y_set[self.dock_wp_idx+1] = self.dock_wp_y_set[1, 0]
                self.odom_wp_x_set[self.dock_wp_idx+2] = self.dock_wp_x_set[2, 0]
                self.odom_wp_y_set[self.dock_wp_idx+2] = self.dock_wp_y_set[2, 0]
            elif msg.data[1] == 0:
                self.get_logger().info("Changing waypoint to docking point: Middle")
                self.odom_wp_x_set[self.dock_wp_idx] = self.dock_wp_x_set[0, 1]
                self.odom_wp_y_set[self.dock_wp_idx] = self.dock_wp_y_set[0, 1]
                self.odom_wp_x_set[self.dock_wp_idx+1] = self.dock_wp_x_set[1, 1]
                self.odom_wp_y_set[self.dock_wp_idx+1] = self.dock_wp_y_set[1, 1]
                self.odom_wp_x_set[self.dock_wp_idx+2] = self.dock_wp_x_set[2, 0]
                self.odom_wp_y_set[self.dock_wp_idx+2] = self.dock_wp_y_set[2, 0]
            elif msg.data[1] == 1:
                self.get_logger().info("Changing waypoint to docking point: Right")
                self.odom_wp_x_set[self.dock_wp_idx] = self.dock_wp_x_set[0, 2]
                self.odom_wp_y_set[self.dock_wp_idx] = self.dock_wp_y_set[0, 2]
                self.odom_wp_x_set[self.dock_wp_idx+1] = self.dock_wp_x_set[1, 2]
                self.odom_wp_y_set[self.dock_wp_idx+1] = self.dock_wp_y_set[1, 2]
                self.odom_wp_x_set[self.dock_wp_idx+2] = self.dock_wp_x_set[2, 0]
                self.odom_wp_y_set[self.dock_wp_idx+2] = self.dock_wp_y_set[2, 0]

            # print(self.odom_wp_x_set)
            # print(self.odom_wp_y_set)

            

    def pub_des(self):
        # print(self.odom_received, self.heading_received, self.spd_received, self.obs_labels_received, self.enu_wp_x_received)
        if (
            # self.odom_received
            # self.robot_pose_received
            self.enu_pos_received
        ):  # all topic received
            print("all topic received now, obs info check")
            if len(self.obs_labels) == len(self.obs_r) \
                and len(self.obs_labels) == len(self.obs_phi) \
                and len(self.obs_labels) == len(self.obs_x) \
                and len(self.obs_labels) == len(self.obs_y) :

                #wp idx => 0~7
                ##phase 1 => CA : wp_idx = 0,1 ,6,7
                ##phase 2 => canal : wp_idx = 2,3,4
                
                #Phase 1 : normal operation
                if self.cur_wp_idx == 0 or self.cur_wp_idx == 1 or self.cur_wp_idx == 5 or self.cur_wp_idx == 6 or self.cur_wp_idx == 7:
                    self.ref_spd = 0.8
                    self.wp_stay_time = 30
                #Phase 2 : canal
                
                elif self.cur_wp_idx == 2 or self.cur_wp_idx == 4:
                    # self.ref_spd = 1.2
                    self.wp_stay_time = 15
                elif self.cur_wp_idx == 3:
                    self.wp_stay_time = 0
                
                #calculate des_heading
                self.cal_des()


                # decrease inf_length
                while len(self.safe_heading) == 0 and self.inf_length > 0.5 * self.B:
                    self.get_logger().info("decreasing inf_length..." + str(self.inf_length) + " -> " + str(self.inf_length - self.B * 0.1))
                    # print("before inf length %d" %self.inf_length)
                    self.inf_length -= 0.1 * self.B
                    # print("decrease inf")
                    # print("current inf length %d" %self.inf_length)
                    self.cal_des()

                # decrease safe_radius
                while len(self.safe_heading) == 0 and self.inf_length <= 0.5 * self.B and self.safe_radius > 0.5:
                    self.get_logger().info("decreasing safe_radius..." + str(self.safe_radius) + " -> " + str(self.safe_radius - 0.1))
                    self.safe_radius -= 0.1
                    self.cal_des()
                    
                print("inf_length: " + str(self.inf_length))
                print("safe_radius: " + str(self.safe_radius))

                if self.inf_length < self.ref_inf_length:
                    self.inf_length = self.ref_inf_length

                if self.safe_radius < self.ref_safe_radius:
                    self.safe_radius = self.ref_safe_radius

                print("safe_heading_size: " + str(len(self.safe_heading)))

            else:
                pass       
            
        else:   # topic not received yet
            pass
        # cur_ort = self.odom_orientation
        # cur_ort = self.robot_orientation
        # euler = quaternion_to_euler(cur_ort)
        #rad
        # self.heading = np.append(self.heading, )
        # self.heading = self.heading[1:]


        

        # wp_cnt increase
        if self.wp_reach_check == True:
            if self.wp_clear == True:
                pass
            else:
                if self.wp_state == False:
                    pass
                else:
                    if self.wp_time_cnt < self.wp_stay_time:
                        self.wp_time_cnt += 1
                    else:
                        self.wp_time_cnt == 0

        # wp_set_x_pub, wp_set_y_pub
        wp_set_x = Float64MultiArray()
        wp_set_x.data = self.odom_wp_x_set.tolist()
        self.wp_set_x_pub.publish(wp_set_x)

        wp_set_y = Float64MultiArray()
        wp_set_y.data = self.odom_wp_y_set.tolist()
        self.wp_set_y_pub.publish(wp_set_y)

        #error heading
        temp_err = self.des_heading[-1] - self.heading[-1]

        if temp_err > np.pi:
            temp_err -= 2*np.pi
        elif temp_err < -np.pi:
            temp_err += 2*np.pi

        self.err_heading = np.append(self.err_heading, temp_err)
        self.err_heading = self.err_heading[1:]
        # cur_pos = self.odom_pos
        # cur_pos = self.robot_pos
        
        if self.enu_pos_received == True:
            cur_pos = self.enu_pos
                    #error x,y
            # transform {global} => {body}
            err_g =np.array([cur_pos[0]-self.odom_wp_x_set[self.cur_wp_idx],
                cur_pos[1]-self.odom_wp_y_set[self.cur_wp_idx],
               self.err_heading[-1]]).transpose()
        
            #rotation matrix
            Rt = np.array([[np.cos(self.heading[-1]), np.sin(self.heading[-1]),0],
                [-np.sin(self.heading[-1]),np.cos(self.heading[-1]),0],
                [0, 0, 1]])
            # print(np.shape(err_g))        
            err_b = np.dot(np.linalg.inv(Rt),err_g)
            # print(err_b)
            temp_err_x = Float64()
            temp_err_x.data = float(err_b[0])
            temp_err_y = Float64()
            temp_err_y.data = float(err_b[1])

            self.err_x_pub.publish(temp_err_x)
            self.err_y_pub.publish(temp_err_y)

            print("cur_wp_idx", self.cur_wp_idx, \
                "des_heading", np.rad2deg(self.des_heading[-1]), \
                "heading", np.rad2deg(self.heading[-1]), \
                "err_heading", np.ceil(np.rad2deg(self.err_heading[-1])), \
                "des_spd", self.des_spd[-1],\
                "wp_check", self.wp_reach_check)
            
            ref_heading = Float64()
            ref_heading.data= self.ref_heading
            self.ref_heading_pub.publish(ref_heading)

            des_heading = Float64()
            des_heading.data = self.des_heading[-1]
            self.des_heading_pub.publish(des_heading)

            err_heading_temp = Float64()
            err_heading_temp.data = self.err_heading[-1]
            self.err_heading_pub.publish(err_heading_temp)
                
            des_spd = Float64()
            des_spd.data = self.des_spd[-1]
            self.des_spd_pub.publish(des_spd)

            cur_wp_idx_ = Int8()
            cur_wp_idx_.data = self.cur_wp_idx
            self.cur_wp_idx_pub.publish(cur_wp_idx_)

            wp_check = Bool()
            wp_check.data = self.wp_reach_check
            self.wp_check_pub.publish(wp_check)

            if self.cur_wp_idx + 1 != len(self.odom_wp_x_set):
                err_heading_next = Float64()
                err_heading_next_ = np.arctan2(self.odom_wp_y_set[self.cur_wp_idx+1] \
                                                - cur_pos[1], self.odom_wp_x_set[self.cur_wp_idx+1] - cur_pos[0]) - self.heading[-1]
                if err_heading_next_ > np.pi:
                    err_heading_next_ -= 2*np.pi
                elif err_heading_next_ < -np.pi:
                    err_heading_next_ += 2*np.pi

                err_heading_next.data = err_heading_next_
                self.err_heading_next_pub.publish(err_heading_next)

        # publish docking state 
        # heading to docking waypoint
        # reached waypoint below docking
        if self.cur_wp_idx == self.dock_wp_idx: # value is arbitrary
            self.docking_state = True
        # docking waypoint passed or not yet
        else:
            self.docking_state = False

        docking_state = Bool()
        docking_state.data = self.docking_state
        self.docking_state_pub.publish(docking_state)

        #docking target_shape 
        target_shape = String()
        target_shape.data = self.target_shape
        # self.cur_wp_idx += 1
        self.target_shape_pub.publish(target_shape)

    def cal_des(self):
        # if(self.odom_received==True):
        # if(self.robot_pose_received==True):
        if self.enu_pos_received == True:

            # cur_pos = self.odom_pos
            # cur_pos = self.robot_pos
            cur_pos = self.enu_pos
            self.print_dist = np.linalg.norm([cur_pos[0] - self.odom_wp_x_set[self.cur_wp_idx], 
                                              cur_pos[1] - self.odom_wp_y_set[self.cur_wp_idx]])
            if self.cur_wp_idx == self.dock_wp_idx or self.cur_wp_idx == self.dock_wp_idx+1:
                self.goal_tol = self.dock_goal_tol
            else:
                self.goal_tol = self.ref_goal_tol


            self.wp_reach_check = bool(
                self.print_dist < self.goal_tol
            )


            #DP later...
            #waypoint reach
            if self.wp_reach_check == True:
                # Waypoint mission clear check
                if self.cur_wp_idx+1 == len(self.odom_wp_x_set):
                    self.get_logger().info("Waypoint Mission Clear")
                    self.wp_clear = True
                else:
                    self.wp_clear = False

                #waypoint mission clear
                if self.wp_clear ==True:
                    print("end")
                
                # on going mission
                else:
                    #first reach in waypoint
                    if self.wp_state == False:
                        self.wp_state = True
                        # self.cur_wp_idx += 1
                        self.des_spd = np.append(self.des_spd, 0)
                        self.des_spd = self.des_spd[1:]
                        des_heading = np.arctan2(self.odom_wp_y_set[self.cur_wp_idx+1] - cur_pos[1], self.odom_wp_x_set[self.cur_wp_idx+1] - cur_pos[0])

                        self.des_heading = np.append(self.des_heading, des_heading)
                        self.des_heading = self.des_heading[1:]
                        # self.wp_time_cnt += 1
                        print("reached")
                        
                    else:  # self.wp_state = True
                        #running while in Docking waypoint
                        if self.docking_state ==True:
                            if self.detect_received == False :
                                self.wp_state = True # -> self.rotate in pwm_cvt
                                self.des_spd = np.append(self.des_spd, 0)
                                self.des_spd = self.des_spd[1:]

                                # heading to the middle image! (2023.08.06)
                                des_heading = np.arctan2(self.odom_wp_y_set[self.cur_wp_idx] - cur_pos[1], self.odom_wp_x_set[self.cur_wp_idx] - cur_pos[0])

                                self.des_heading = np.append(self.des_heading, des_heading)
                                self.des_heading = self.des_heading[1:]
                            #shape received
                            #head to dock position
                            else : 
                                self.wp_state = False
                                self.des_spd = np.append(self.des_spd, self.ref_spd)
                                self.des_spd = self.des_spd[1:]
                                des_heading = np.arctan2(self.odom_wp_y_set[self.cur_wp_idx] - cur_pos[1], self.odom_wp_x_set[self.cur_wp_idx] - cur_pos[0])

                                self.des_heading = np.append(self.des_heading, des_heading)
                                self.des_heading = self.des_heading[1:]
                                # self.cur_wp_idx += 1


                        else:
                            #running while in waypoint
                            if self.wp_time_cnt < self.wp_stay_time:
                                # self.wp_time_cnt += 1
                                self.wp_state = True
                                self.des_spd = np.append(self.des_spd, 0)
                                self.des_spd = self.des_spd[1:]
                                des_heading = np.arctan2(self.odom_wp_y_set[self.cur_wp_idx+1] - cur_pos[1], self.odom_wp_x_set[self.cur_wp_idx+1] - cur_pos[0])

                                self.des_heading = np.append(self.des_heading, des_heading)
                                self.des_heading = self.des_heading[1:]

                            #exit waypoint
                            else:  # self.wp_time_cnt > self.wp_stay_time
                                # self.wp_time_cnt = 0
                                self.wp_state = False
                                self.des_spd = np.append(self.des_spd, self.ref_spd)
                                self.des_spd = self.des_spd[1:]
                                des_heading = np.arctan2(self.odom_wp_y_set[self.cur_wp_idx+1] - cur_pos[1], self.odom_wp_x_set[self.cur_wp_idx+1] - cur_pos[0])

                                self.des_heading = np.append(self.des_heading, des_heading)
                                self.des_heading = self.des_heading[1:]
                                self.get_logger().info("Changing waypoint ...")
                                self.cur_wp_idx += 1
                        
            #waypoing going
            else:  # wp_reach_check == False:
                
                
                self.wp_state = False
                print("this is cur_wp_idx")
                print(self.cur_wp_idx)
                self.ref_heading = np.arctan2(self.odom_wp_y_set[self.cur_wp_idx] - cur_pos[1], self.odom_wp_x_set[self.cur_wp_idx] - cur_pos[0])
                print("this is ref heading")
                print(np.rad2deg(self.ref_heading))
                #### calculate des_heading and des_spd
                if len(self.obs_labels) != 0: # if there are scanned obstacles
                    self.danger_r = []
                    self.danger_phi = []
                    self.danger_x = []
                    self.danger_y = []
                    self.safe_phi = np.linspace(-np.pi, np.pi, 360).transpose()
                    # print(np.shape(self.safe_phi))
                    self.safe_heading = []

                    # safe_phi
                    idx_array = np.where(np.diff(self.obs_labels) != 0)[0] + 1
                    # idx_array = np.where(np.diff(self.obs_labels) != 0)[0]
                    idx_array =np.append(0,idx_array)
                    # print(len(idx_array))
                    # print(idx_array)
                    # print("this is r fro obs")
                    # print(self.obs_r)
                    #cal safe_heading
                    for i, idx in enumerate(idx_array):
                        #loop until last obs
                        if idx != idx_array[-1]:
                            # print(len(self.obs_r[idx:idx_array[i+1]-1]))
                            # delete noise obstable, setting point num
                            if len(self.obs_r[idx:idx_array[i+1]-1]) > 10:
                                # print(1)
                                # print(self.obs_r[idx:idx_array[i+1]-1])
                                # print("this is minimum value")
                                # print(np.min(self.obs_r[idx:idx_array[i+1]-1]))

                                # print("this is obs_phi")
                                # print(np.rad2deg(self.obs_phi[idx:idx_array[i+1]-1]))
                                if np.min(self.obs_r[idx:idx_array[i+1]-1]) < self.safe_radius:
                                    # define start_phi
                                    r = self.obs_r[idx]
                                    inflate_obs_phi = np.arccos((2*r*r - (self.B * 1.2)**2)/(2*r*r))
                                    del r

                                    if self.obs_phi[idx] - inflate_obs_phi > -np.pi:
                                        start_phi = self.obs_phi[idx] - inflate_obs_phi
                                    else:
                                        start_phi = -np.pi

                                    del inflate_obs_phi

                                    # define end_phi
                                    r = self.obs_r[idx_array[i+1]-1]
                                    inflate_obs_phi = np.arccos((2*r*r - (self.B * 1.2)**2)/(2*r*r)) # TODO: adjust parameters depending on dimension
                                    del r

                                    if self.obs_phi[idx_array[i+1]-1] + inflate_obs_phi < np.pi:
                                        end_phi = self.obs_phi[idx_array[i+1]-1] + inflate_obs_phi
                                    else:
                                        end_phi = np.pi

                                    del inflate_obs_phi

                                    # print("start_phi, end_phi")
                                    # print(np.rad2deg(start_phi), np.rad2deg(end_phi))
                                    temp_safe_phi1 = self.safe_phi
                                    safe_idx1 = np.array(temp_safe_phi1 < start_phi)
                                    temp_safe_phi1 = temp_safe_phi1[safe_idx1]
                                    temp_safe_phi1 = np.reshape(temp_safe_phi1, (1, -1))
                                    # print("start~")
                                    # print(np.rad2deg(temp_safe_phi1))
                                    temp_safe_phi2 = self.safe_phi
                                    safe_idx2 = np.array(temp_safe_phi2 > end_phi)
                                    temp_safe_phi2= temp_safe_phi2[safe_idx2]
                                    temp_safe_phi2 = np.reshape(temp_safe_phi2, (1, -1))
                                    # print("~end")
                                    # print(np.rad2deg(temp_safe_phi2))
                                    self.safe_phi = np.append(temp_safe_phi1, temp_safe_phi2)
                                    # print("this is safe_phi")
                                    # print(np.rad2deg(self.safe_phi))
                                    # return 0
                            
                        else:
                            if len(self.obs_r[idx:]) > 10:
                                # print(2)
                                if np.min(self.obs_r[idx:]) < self.safe_radius:
                                    # print(2)
                                    # define start_phi
                                    r = self.obs_r[idx]
                                    inflate_obs_phi = np.arccos((2*r*r - (self.inf_length)**2)/(2*r*r))
                                    del r
                                    # print("this is inflate obs phi")
                                    # print(inflate_obs_phi)
                                    # print("this is last obs phi")
                                    # print(np.rad2deg(self.obs_phi[idx:]))
                                    if self.obs_phi[idx] - inflate_obs_phi > -np.pi:
                                        start_phi = self.obs_phi[idx] - inflate_obs_phi
                                    else:
                                        start_phi = -np.pi

                                    del inflate_obs_phi

                                    # define end_phi
                                    r = self.obs_r[-1]
                                    inflate_obs_phi = np.arccos((2*r*r - (self.inf_length)**2)/(2*r*r))
                                    del r

                                    if self.obs_phi[-1] + self.inflate_obs_phi < np.pi:
                                        end_phi = self.obs_phi[-1] + self.inflate_obs_phi
                                    else:
                                        end_phi = np.pi

                                    del inflate_obs_phi

                                    # print(np.rad2deg(start_phi), np.rad2deg(end_phi))
                                    temp_safe_phi1 = self.safe_phi
                                    safe_idx1 = np.array(temp_safe_phi1 < start_phi)
                                    temp_safe_phi1 = temp_safe_phi1[safe_idx1]
                                    temp_safe_phi1 = np.reshape(temp_safe_phi1, (1, -1))
                                    # print("this is last obs ~start")
                                    # print(np.rad2deg(temp_safe_phi1))
                                    temp_safe_phi2 = self.safe_phi
                                    safe_idx2 = np.array(temp_safe_phi2 >end_phi)
                                    temp_safe_phi2= temp_safe_phi2[safe_idx2]
                                    temp_safe_phi2 = np.reshape(temp_safe_phi2, (1, -1))
                                    # print("this is last obs end~")
                                    # print(np.rad2deg(temp_safe_phi2))
                                    self.safe_phi = np.append(temp_safe_phi1, temp_safe_phi2)
                                    
                    # print("safe_phi")
                    # print(np.rad2deg(self.safe_phi))
                    # self.safe_heading = self.safe_phi
                    self.safe_heading = self.safe_phi + self.heading[-1]
                    # print("safe heading size: " + str(len(self.safe_heading)))
                    # print(self.safe_heading)
                    minus_idx = self.safe_heading > np.pi
                    self.safe_heading[minus_idx] -= 2*np.pi
                    plus_idx = self.safe_heading < -np.pi
                    self.safe_heading[plus_idx] += 2*np.pi
                    # print("deg safe heading")
                    # print(np.rad2deg(self.safe_heading))
                    ##safe phi

                    future_cost = self.safe_heading - self.ref_heading
                    future_minus_idx = future_cost > np.pi
                    future_cost[future_minus_idx] -= 2*np.pi
                    future_plus_idx = future_cost < -np.pi
                    future_cost[future_plus_idx] += 2*np.pi

                    past_cost = self.safe_heading - self.des_heading[-1]
                    past_minus_idx = past_cost > np.pi
                    past_cost[past_minus_idx] -= 2*np.pi
                    past_plus_idx = past_cost < -np.pi
                    past_cost[past_plus_idx] += 2*np.pi

                    # print(future_cost)
                    
                    # past_cost=0

                    #ds heading
                    self.heading_cost = abs(future_cost) + 0.2 * abs(past_cost)
                    # self.heading_cost = abs(future_cost)
                    # print(self.heading_cost)
                    # print(np.rad2deg(self.heading_cost))

                    # calculate des_heading
                    
                    self.des_spd = np.append(self.des_spd, self.ref_spd)
                    self.des_spd = self.des_spd[1:]
                    temp_safe = Float64MultiArray()
                    temp_safe.data = (self.safe_heading).tolist()
                    self.safe_heading_pub.publish(temp_safe)
                    # print("this is safe heading")
                    # print(self.safe_heading)
                    if len(self.safe_heading) != 0:
                        des_heading_idx = np.argmin(self.heading_cost)
                        des_heading = self.safe_heading[des_heading_idx]
                        
                        self.des_heading = np.append(self.des_heading, des_heading)
                        self.des_heading = self.des_heading[1:]
                        # if self.safe_radius < self.ref_safe_radius:
                        #     self.safe_radius += 0.1 # increase safe_radius until it reaches self.ref_safe_radius

                        # # Roll back safe values 
                        # if self.inf_length < self.ref_inf_length:
                        #     self.inf_length += 0.01
                        # if self.safe_radius < self.ref_safe_radius:
                        #     self.safe_radius += 0.01
                        
                    else: # no safe heading
                        #Edit inf length until 0.5 B
                        
                        # if self.inf_length > 0.5 * self.B:
                        #     self.inf_length -= 0.01

                        # # inf length not enough
                        # # Edit safe radius
                        # else: 
                        #     if self.safe_radius > 0.5:
                        #         self.safe_radius -= 0.01
                        #     else:
                        #         self.get_logger().info('No Answer GG')
                        self.get_logger().info('No safe heading')
                        # self.get_logger().info("reduce inf length : " + str(self.inf_length))
                        # self.get_logger().info("reduce safe radius : " + str(self.safe_radius))
                        return
                    
                    # self.get_logger().info('Inf Length: ' + str(self.inf_length))
                    # self.get_logger().info('Safe radius: ' + str(self.safe_radius))

                else: # if there are no scanned obstacles
                    self.des_spd = np.append(self.des_spd, self.des_spd[-1])
                    self.des_spd = self.des_spd[1:]
                    # self.des_heading = np.append(self.des_heading, self.des_heading[-1])
                    # self.des_heading = self.des_heading[1:]
                    self.des_heading = np.append(self.des_heading, self.ref_heading)
                    self.des_heading = self.des_heading[1:]
            
            wp_clear = Bool()
            wp_clear.data = self.wp_clear
            #wp clear index publish
            self.wp_clear_pub.publish(wp_clear)
        else:
            print("not received yet")


        


def quaternion_to_euler(q):
    """
    Convert a quaternion to Euler angles (roll, pitch, yaw).

    :param q: A tuple or list representing the quaternion (w, x, y, z).
    :return: A tuple representing the Euler angles (roll, pitch, yaw) in radians.
    """
    # w, x, y, z = q

    x = q[0]
    y = q[1]
    z = q[2]
    w = q[3]

    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = np.arcsin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance = Obstacle_Avoidance()
    obstacle_avoidance.wait_for_topics()
    rclpy.spin(obstacle_avoidance)
    obstacle_avoidance.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
