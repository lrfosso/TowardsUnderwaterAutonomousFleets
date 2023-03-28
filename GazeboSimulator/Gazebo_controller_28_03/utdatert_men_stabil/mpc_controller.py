import sys
import os
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path)
#sys.path.append("/home/tor/bluerovSim/bluerov2_garden/install/mpc_controller/lib/python3.10/site-packages/mpc_controller")
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
from geometry_msgs.msg import Vector3
from module_rovModel import *
from module_rovController import *
from time import sleep

# ########################################## MULTI LAUNCH + pubsub kode fra 26/03 ############################################################
class BluerovPubSubNode(Node):
    def vector_between_rovs(self,x1,y1,z1,x2,y2,z2):
        x = (x2-x1)
        y = (y2-y1)
        z = (z2-z1)
        return [x, y, z]
    def z_directional_vector_from_quaternion(self, q0, e1, e2, e3):
        x = -2*(e1*e3+e2*q0)
        y = -2*(e2*e3-e1*q0)
        z = -1+2*(e1**2+e2**2)
        return [x, y, z]

    def x_directional_vector_from_quaternion(self, q0, e1, e2, e3):
        x = 1-2*(e2**2+e3**2)
        y = 2*(e1*e2+e3*q0)
        z = 2*(e1*e3-e2*q0)
        return [x, y, z]

    def __init__(self):
        self.ready = False
        super().__init__('bluerov_pubsub')
        #self.declare_parameter('model_name')
        #self.rov_name = self.get_parameter('model_name').get_parameter_value().string_value

        self.declare_parameter('id')
        self.declare_parameter('fleets-id')
        self.main_id = self.get_parameter('id').get_parameter_value().string_value
        self.fleet_id = self.get_parameter('fleets-id').get_parameter_value().string_value


    #Init code
        self.u0_1 = np.array([[0],[0],[0],[0],[0],[0],[0],[0]])
        self.odometry_list = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0])
        self.odometry_2_list = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0])

        self.modelRov1 = MyROVModel()
        self.mpc1 = MyController(self.modelRov1,4)
        self.x0_1 = np.array([0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,0]).reshape(-1,1)
        self.mpc1.x0 = self.x0_1
        self.i = 0


        #self.estimator1 = do_mpc.estimator.StateFeedback(self.modelRov1.model)
        #self.simulator1 = do_mpc.simulator.Simulator(self.modelRov1.model)
        #self.tvp_template1 = self.simulator1.get_tvp_template()
        
        #self.simulator1.set_tvp_fun(self.tvp_template1)

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

        #self.q_0_sp.data = 0.9239557
        #self.e_1_sp.data = 0.0
        #self.e_2_sp.data = 0.0
        #self.e_3_sp.data = 0.3824995



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
            "/bluerov2_pid/bluerov{}/observer/nlo/odom_ned".format(self.main_id), #Topic
            self.listener_callback, #function?
            10)
        
        self.odome_2_subscriber = self.create_subscription(  
            Odometry, #Message type
            "/bluerov2_pid/bluerov{}/observer/nlo/odom_ned".format(self.fleet_id), #Topic
            self.listener_2_callback, #function?
            10)
        self.ref_subscriber = self.create_subscription(  
            Vector3, #Message type
            "/ref", #Topic
            self.ref_callback, #function?
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


       # self.x_sp_subscriber
       # self.y_sp_subscriber
       # self.z_sp_subscriber
        self.q_0_sp_subscriber
        self.e_1_sp_subscriber
        self.e_2_sp_subscriber
        self.e_3_sp_subscriber

        self.publisher_1 = self.create_publisher(Float64, '/model/bluerov{}/joint/thruster1_joint/cmd_thrust'.format(self.main_id), 10)
        self.publisher_2 = self.create_publisher(Float64, '/model/bluerov{}/joint/thruster2_joint/cmd_thrust'.format(self.main_id), 10)
        self.publisher_3 = self.create_publisher(Float64, '/model/bluerov{}/joint/thruster3_joint/cmd_thrust'.format(self.main_id), 10)
        self.publisher_4 = self.create_publisher(Float64, '/model/bluerov{}/joint/thruster4_joint/cmd_thrust'.format(self.main_id), 10)
        self.publisher_5 = self.create_publisher(Float64, '/model/bluerov{}/joint/thruster5_joint/cmd_thrust'.format(self.main_id), 10)
        self.publisher_6 = self.create_publisher(Float64, '/model/bluerov{}/joint/thruster6_joint/cmd_thrust'.format(self.main_id), 10)
        self.publisher_7 = self.create_publisher(Float64, '/model/bluerov{}/joint/thruster7_joint/cmd_thrust'.format(self.main_id), 10)
        self.publisher_8 = self.create_publisher(Float64, '/model/bluerov{}/joint/thruster8_joint/cmd_thrust'.format(self.main_id), 10) 
        self.angle_publisher = self.create_publisher(Float64, '/angle_{}_to_{}'.format(self.main_id, self.fleet_id), 10)     
        
        
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

        self.ready = True

#    def x_sp_callback(self, msg):
#        self.mpc1.x_setp = msg.data
#
#    def y_sp_callback(self, msg):
#        self.mpc1.y_setp = msg.data
#
#    def z_sp_callback(self, msg):
#        self.mpc1.z_setp = msg.data
    def ref_callback(self,msg):
        self.mpc1.x_setp = msg.x
        self.mpc1.y_setp = msg.y
        self.mpc1.z_setp = msg.z

        this_rov_pos = [float(self.odometry_list[0]), float(self.odometry_list[1]), float(self.odometry_list[2])]
        other_rov_pos = [float(self.odometry_2_list[0]), float(self.odometry_2_list[1]), float(self.odometry_2_list[2])]


        vector = np.array(other_rov_pos) - np.array(this_rov_pos)
        vector = vector / np.linalg.norm(vector)

        # Calculate the quaternion angles
        theta = np.arccos(np.dot([1, 0, 0], vector))
        axis = np.cross([1, 0, 0], vector)
        if(sum(axis) != 0):
            #print("#######################{} [{},{},{}] #######################".format(self.fleet_id, round(other_rov_pos[0],2), round(other_rov_pos[1],2), round(other_rov_pos[2],2)))
            #print("#######################{} [{},{},{}] #######################".format(self.main_id, round(this_rov_pos[0],2), round(this_rov_pos[1],2), round(this_rov_pos[2],2)))
            #print("####################### Mellom [{},{},{}] #######################".format(round(vector[0],2), round(vector[1],2), round(vector[2],2)))
            axis = axis / np.linalg.norm(axis)

            self.mpc1.q_0_setp = np.cos(theta/2)
            self.mpc1.e_1_setp = axis[0] * np.sin(theta/2)
            self.mpc1.e_2_setp = axis[1] * np.sin(theta/2)
            self.mpc1.e_3_setp = axis[2] * np.sin(theta/2)
            print("######## QUART SP til {}: [{}, {}, {}, {}]".format(self.main_id, self.mpc1.q_0_setp, self.mpc1.e_1_setp, self.mpc1.e_2_setp, self.mpc1.e_3_setp))

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
        print(self.u0_1)
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
        
        if(self.ready):
            x1 = self.odometry_list[0]
            y1 = self.odometry_list[1]
            z1 = self.odometry_list[2]

            x2 = self.odometry_2_list[0]
            y2 = self.odometry_2_list[1]
            z2 = self.odometry_2_list[2]

            q0 = self.odometry_list[3]
            e1 = self.odometry_list[4]
            e2 = self.odometry_list[5]
            e3 = self.odometry_list[6]

            v1 = self.vector_between_rovs(x1, y1, z1, x2, y2, z2)
            v2 = self.x_directional_vector_from_quaternion(q0, e1, e2, e3)


            angle = Float64()
            #angle.x,angle.y,angle.z = v2[0],v2[1],v2[2]
            angle.data = ((np.arccos((np.dot(v1, v2))/(np.linalg.norm(v1)*np.linalg.norm(v2))))/np.pi)*180
            self.angle_publisher.publish(angle)
            

def main(args=None):
    rclpy.init(args=args)
    bluerov_pubsub_node = BluerovPubSubNode()
    rclpy.spin(bluerov_pubsub_node)
    rclpy.shutdown()


main()


