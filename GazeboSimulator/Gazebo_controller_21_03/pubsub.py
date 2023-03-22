import sys
sys.path.append("/home/rov/Downloads/bluerov2_garden/install/pubsub_template/lib/python3.10/site-packages/pubsub_template")
## 
#tor@PC:~/bluerovSim/ver0803/bluerov2_garden/src$ ls


import numpy as np
import matplotlib.pyplot as plt
import sys
from casadi import *

import do_mpc
##

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from rovModel import *
from rovController import *
from time import sleep

class Bluerov2PubSubNode(Node):

    def calculate_quaternion(self, current_position, x_sp, y_sp, z_sp):
        # Calculate the vector pointing from current position to desired position
        
        desired_position = np.array([x_sp,y_sp,z_sp])
        current_position
        vector = np.array(desired_position) - np.array(current_position[:3])
        vector = vector / np.linalg.norm(vector)
    
        # Calculate the quaternion angles
        theta = np.arccos(np.dot([0, 0, 1], vector))
        axis = np.cross([0, 0, 1], vector)
        axis = axis / np.linalg.norm(axis)
    
        w = np.cos(theta/2)
        x = axis[0] * np.sin(theta/2)
        y = axis[1] * np.sin(theta/2)
        z = axis[2] * np.sin(theta/2)
    
        return [w, x, y, z]
 
    def __init__(self):
        super().__init__('bluerov2_pubsub')

    #Init code
        self.u0_1 = np.array([[0],[0],[0],[0],[0],[0],[0],[0]])
        self.odometry_list = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0])

        self.modelRov1 = MyROVModel()
        self.mpc1 = MyController(self.modelRov1,5)
        self.x0_1 = np.array([0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,0]).reshape(-1,1)
        self.mpc1.x0 = self.x0_1

        self.estimator1 = do_mpc.estimator.StateFeedback(self.modelRov1.model)
        self.simulator1 = do_mpc.simulator.Simulator(self.modelRov1.model)
        self.tvp_template1 = self.simulator1.get_tvp_template()
        
        self.simulator1.set_tvp_fun(self.tvp_template1)

        self.mpc1.mpc.set_initial_guess()

        # init setpoint
        self.x_sp = Float64()
        self.y_sp = Float64()
        self.z_sp = Float64()
        self.q_0_sp = Float64()
        self.e_1_sp = Float64()
        self.e_2_sp = Float64()
        self.e_3_sp = Float64()

        self.x_sp.data = 0.0
        self.y_sp.data = 0.0
        self.z_sp.data = 2.0

        self.q_0_sp.data = 0.707
        self.e_1_sp.data = 0.0
        self.e_2_sp.data = 0.0
        self.e_3_sp.data = 0.707
        
        self.mpc1.x_setp =  self.x_sp.data
        self.mpc1.y_setp =  self.y_sp.data
        self.mpc1.z_setp =  self.z_sp.data

        self.mpc1.q_0_setp = self.q_0_sp.data
        self.mpc1.e_1_setp = self.e_1_sp.data
        self.mpc1.e_2_setp = self.e_2_sp.data
        self.mpc1.e_3_setp = self.e_3_sp.data

        #Creating a subscriber
        self.odome_subscriber = self.create_subscription(  
            Odometry, #Message type
            "/bluerov2_pid/bluerov2/observer/nlo/odom_ned", #Topic
            self.listener_callback, #function?
            10)
        
        self.odome_2_subscriber = self.create_subscription(  
            Odometry, #Message type
            "/bluerov2_pid/bluerov3/observer/nlo/odom_ned", #Topic
            self.listener_2_callback, #function?
            10) 
        self.x_sp_subscriber = self.create_subscription(  
            Float64, #Message type
            "/xsp", #Topic
            self.x_sp_callback, #function?
            10)
        self.y_sp_subscriber = self.create_subscription(  
            Float64, #Message type
            "/ysp", #Topic
            self.y_sp_callback, #function?
            10)
        self.z_sp_subscriber = self.create_subscription(  
            Float64, #Message type
            "/zsp", #Topic
            self.z_sp_callback, #function?
            10)
        self.q_0_sp_subscriber = self.create_subscription(  
            Float64, #Message type
            "/q_0sp", #Topic
            self.q_0_sp_callback, #function?
            10)
        self.e_1_sp_subscriber = self.create_subscription(  
            Float64, #Message type
            "/e_1sp", #Topic
            self.e_1_sp_callback, #function?
            10)
        self.e_2_sp_subscriber = self.create_subscription(  
            Float64, #Message type
            "/e_2sp", #Topic
            self.e_2_sp_callback, #function?
            10)
        self.e_3_sp_subscriber = self.create_subscription(  
            Float64, #Message type
            "/e_3sp", #Topic
            self.e_3_sp_callback, #function?
            10)
        self.odome_subscriber #Prevent unused variable warning
        self.odome_2_subscriber 


        self.x_sp_subscriber
        self.y_sp_subscriber
        self.z_sp_subscriber
        self.q_0_sp_subscriber
        self.e_1_sp_subscriber
        self.e_2_sp_subscriber
        self.e_3_sp_subscriber

        self.publisher_1 = self.create_publisher(Float64, '/model/bluerov2/joint/thruster1_joint/cmd_thrust', 10)
        self.publisher_2 = self.create_publisher(Float64, '/model/bluerov2/joint/thruster2_joint/cmd_thrust', 10)
        self.publisher_3 = self.create_publisher(Float64, '/model/bluerov2/joint/thruster3_joint/cmd_thrust', 10)
        self.publisher_4 = self.create_publisher(Float64, '/model/bluerov2/joint/thruster4_joint/cmd_thrust', 10)
        self.publisher_5 = self.create_publisher(Float64, '/model/bluerov2/joint/thruster5_joint/cmd_thrust', 10)
        self.publisher_6 = self.create_publisher(Float64, '/model/bluerov2/joint/thruster6_joint/cmd_thrust', 10)
        self.publisher_7 = self.create_publisher(Float64, '/model/bluerov2/joint/thruster7_joint/cmd_thrust', 10)
        self.publisher_8 = self.create_publisher(Float64, '/model/bluerov2/joint/thruster8_joint/cmd_thrust', 10)        
        
        
        # #Calls on the talker_callback function every 0.1
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.talker_callback)

    def listener_callback(self, msg):    
        self.odometry_list = [msg.pose.pose.position.x,
                              msg.pose.pose.position.y,
                              msg.pose.pose.position.z,
                              msg.pose.pose.orientation.w,
                              msg.pose.pose.orientation.x,
                              msg.pose.pose.orientation.y,
                              msg.pose.pose.orientation.z,
                              msg.twist.twist.linear.x,
                              msg.twist.twist.linear.y,
                              msg.twist.twist.linear.z,
                              msg.twist.twist.angular.x,
                              msg.twist.twist.angular.y,
                              msg.twist.twist.angular.z]
        self.x0_1 = np.array(self.odometry_list)

    def listener_2_callback(self, msg):    
        self.odometry_2_list = [msg.pose.pose.position.x,
                              msg.pose.pose.position.y,
                              msg.pose.pose.position.z,
                              msg.pose.pose.orientation.w,
                              msg.pose.pose.orientation.x,
                              msg.pose.pose.orientation.y,
                              msg.pose.pose.orientation.z,
                              msg.twist.twist.linear.x,
                              msg.twist.twist.linear.y,
                              msg.twist.twist.linear.z,
                              msg.twist.twist.angular.x,
                              msg.twist.twist.angular.y,
                              msg.twist.twist.angular.z]
        self.x0_2 = np.array(self.odometry_2_list)

        self.mpc1.x_2 = self.x0_2[0]
        self.mpc1.y_2 = self.x0_2[1]
        self.mpc1.z_2 = self.x0_2[2]

    def x_sp_callback(self, msg):
        self.mpc1.x_setp = msg.data

    def y_sp_callback(self, msg):
        self.mpc1.y_setp = msg.data

    def z_sp_callback(self, msg):
        self.mpc1.z_setp = msg.data

    def q_0_sp_callback(self, msg):
        self.mpc1.q_0_setp = msg.data

    def e_1_sp_callback(self, msg):
        self.mpc1.e_1_setp = msg.data

    def e_2_sp_callback(self, msg):
        self.mpc1.e_2_setp = msg.data

    def e_3_sp_callback(self, msg):
        self.mpc1.e_3_setp = msg.data

    def talker_callback(self):
        #self.get_logger().info('\nODOM VECTOR: "%s"' % self.x0_1)
        self.u0_1 = self.mpc1.mpc.make_step(self.x0_1)
        #self.get_logger().info("\nPÅDRAG: \n%s" % self.u0_1)
        thrust1 = Float64()
        thrust1.data = round(float(self.u0_1[0][0]),2)
        thrust2 = Float64()
        thrust2.data = round(float(self.u0_1[1][0]),2)
        thrust3 = Float64()
        thrust3.data = round(float(self.u0_1[2][0]),2)
        thrust4 = Float64()
        thrust4.data = round(float(self.u0_1[3][0]),2)
        thrust5 = Float64()
        thrust5.data = round(float(self.u0_1[4][0]),2)
        thrust6 = Float64()
        thrust6.data = round(float(self.u0_1[5][0]),2)
        thrust7 = Float64()
        thrust7.data = round(float(self.u0_1[6][0]),2)
        thrust8 = Float64()
        thrust8.data = round(float(self.u0_1[7][0]),2)


        #self.get_logger().info('\nTHRUSTERE:\n1:{}\n2:{}\n3:{}\n4:{}\n5:{}\n6:{}\n7:{}\n8:{}'.format(thrust1.data,thrust2.data,thrust3.data,thrust4.data,thrust5.data,thrust6.data,thrust7.data,thrust8.data))
        

        self.publisher_1.publish(thrust1)
        self.publisher_2.publish(thrust2)
        self.publisher_3.publish(thrust3)
        self.publisher_4.publish(thrust4)
        self.publisher_5.publish(thrust5)
        self.publisher_6.publish(thrust6)
        self.publisher_7.publish(thrust7)
        self.publisher_8.publish(thrust8)

        print(self.x0_1)
        #quat_sp = self.calculate_quaternion(self.odometry_list, self.mpc1.x_setp, self.mpc1.y_setp, self.mpc1.z_setp)
        #self.mpc1.q_0_setp = float(quat_sp[0])
        #self.mpc1.e_1_setp = float(quat_sp[1])
        #self.mpc1.e_2_setp = float(quat_sp[2])
        #self.mpc1.e_3_setp = float(quat_sp[3])
       # self.mpc1.q_0_setp = 1
       # self.mpc1.e_1_setp = 0
       # self.mpc1.e_2_setp = 0
       # self.mpc1.e_3_setp = 0


   

def main(args=None):
    rclpy.init(args=args)
    bluerov2_pubsub_node = Bluerov2PubSubNode()
    rclpy.spin(bluerov2_pubsub_node)
    rclpy.shutdown()


main()
