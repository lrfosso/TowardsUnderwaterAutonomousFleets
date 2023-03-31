import sys
import os
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path)
import numpy as np
import sys
from casadi import *
import do_mpc
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from module_rovModel import *
from module_rovController import *
import logging

class BluerovPubSubNode(Node):
    def __init__(self):
        super().__init__('bluerov_pubsub')
        self.ready_signal_mpc = False # Flag to start the MPC
        # Node launch parameters

        # Declare parameters
        # self.declare_parameter('')
        self.declare_parameter('main_id')
        self.declare_parameter('fleet_quantity')
        self.declare_parameter('FOV_constraint')
        self.declare_parameter('radius_setp')
        self.declare_parameter('distance_rovs')
        self.declare_parameter('FOV_range_deg')
        self.declare_parameter('FOV_range_soft_deg')
        self.declare_parameter('cycle_time_publish')


        # Get parameters
        # self. = self.get_parameter('').get_parameter_value().
        self.main_id = self.get_parameter('main_id').get_parameter_value().integer_value
        self.fleet_quantity = self.get_parameter('fleet_quantity').get_parameter_value().integer_value
        self.FOV_constraint = self.get_parameter('FOV_constraint').get_parameter_value().bool_value
        self.radius_setp = self.get_parameter('radius_setp').get_parameter_value().double_value
        self.distance_rovs = self.get_parameter('distance_rovs').get_parameter_value().double_value
        self.FOV_range_deg = self.get_parameter('FOV_range_deg').get_parameter_value().double_value
        self.FOV_range_soft_deg = self.get_parameter('FOV_range_soft_deg').get_parameter_value().double_value
        self.cycle_time_publish = self.get_parameter('cycle_time_publish').get_parameter_value().double_value

        # Initialize the MPC
        #self.FOV_constraint = True
        self.modelRov = MyROVModel()
        self.mpc1 = MyController(self.modelRov, # MPC-controller parameters
                                n_multi_agent=self.fleet_quantity,
                                radius_setp=self.radius_setp,
                                distance_rovs=self.distance_rovs,
                                FOV_range_deg=self.FOV_range_deg,
                                FOV_range_soft_deg=self.FOV_range_soft_deg,
                                FOV_constraint= self.FOV_constraint
                                )

        # Initialize subscribers for the main ROV
        self.main_odometry_subscriber = self.create_subscription(  
            Odometry,
            "/bluerov2_pid/bluerov{}/observer/nlo/odom_ned".format(self.main_id),
            self.main_odemetry_callback, #Callback function
            10)
        self.ref_subscriber = self.create_subscription(  
            Vector3,
            "/ref",
            self.reference_callback, #Callback function
            10) 
        
        #Creating publishers for the thrusters
        self.publisher_1 = self.create_publisher(Float64, '/model/bluerov{}/joint/thruster1_joint/cmd_thrust'.format(self.main_id), 10)
        self.publisher_2 = self.create_publisher(Float64, '/model/bluerov{}/joint/thruster2_joint/cmd_thrust'.format(self.main_id), 10)
        self.publisher_3 = self.create_publisher(Float64, '/model/bluerov{}/joint/thruster3_joint/cmd_thrust'.format(self.main_id), 10)
        self.publisher_4 = self.create_publisher(Float64, '/model/bluerov{}/joint/thruster4_joint/cmd_thrust'.format(self.main_id), 10)
        self.publisher_5 = self.create_publisher(Float64, '/model/bluerov{}/joint/thruster5_joint/cmd_thrust'.format(self.main_id), 10)
        self.publisher_6 = self.create_publisher(Float64, '/model/bluerov{}/joint/thruster6_joint/cmd_thrust'.format(self.main_id), 10)
        self.publisher_7 = self.create_publisher(Float64, '/model/bluerov{}/joint/thruster7_joint/cmd_thrust'.format(self.main_id), 10)
        self.publisher_8 = self.create_publisher(Float64, '/model/bluerov{}/joint/thruster8_joint/cmd_thrust'.format(self.main_id), 10)

        # Multi agent subscribers
        multi_agent_id = [2, 3, 4, 5, 6, 7] #List of ROV IDs
        multi_agent_id.pop(self.main_id - 2) #Remove the main ROV ID from the list

        if(self.fleet_quantity > 1): 
            self.odometry_2_subscriber = self.create_subscription(  
                Odometry, #Message type
                "/bluerov2_pid/bluerov{}/observer/nlo/odom_ned".format(multi_agent_id[-7+self.fleet_quantity]), #Topic
                self.odometry_callback_2, #function?
                10)
            multi_agent_id.pop(-7+self.fleet_quantity)
        if(self.fleet_quantity > 2):
            self.odometry_3_subscriber = self.create_subscription( 
                Odometry, #Message type
                "/bluerov2_pid/bluerov{}/observer/nlo/odom_ned".format(multi_agent_id[0]), #Topic
                self.odometry_callback_3, #function?
                10)
        if(self.fleet_quantity > 3):
            self.odometry_4_subscriber = self.create_subscription( 
                Odometry, #Message type
                "/bluerov2_pid/bluerov{}/observer/nlo/odom_ned".format(multi_agent_id[1]), #Topic
                self.odometry_callback_4, #function?
                10)
        if(self.fleet_quantity > 4):
            self.odometry_5_subscriber = self.create_subscription( 
                Odometry, #Message type
                "/bluerov2_pid/bluerov{}/observer/nlo/odom_ned".format(multi_agent_id[2]), #Topic
                self.odometry_callback_5, #function?
                10)
        if(self.fleet_quantity > 5):
            self.odometry_6_subscriber = self.create_subscription( 
                Odometry, #Message type
                "/bluerov2_pid/bluerov{}/observer/nlo/odom_ned".format(multi_agent_id[3]), #Topic
                self.odometry_callback_6, #function?
                10)
        
        self.main_odometry_subscriber #Prevent unused variable warning
        cycle_time_publish = 0.05  # seconds
        self.timer = self.create_timer(cycle_time_publish, self.publisher_callback)

    def vector_between_rovs(self,x1,y1,z1,x2,y2,z2):
        """Gets xyz coords of ROVs as input. Returns the vector between them (- heave)"""
        x = (x2-x1)
        y = (y2-y1)
        z = (z2-z1)
        return [x, y, z]
    def z_directional_vector_from_quaternion(self, q0, e1, e2, e3):
        """Returns the directional vector of the ROV in -z direction"""
        x = -2*(e1*e3+e2*q0)
        y = -2*(e2*e3-e1*q0)
        z = -1+2*(e1**2+e2**2)
        return [x, y, z]
    def x_directional_vector_from_quaternion(self, q0, e1, e2, e3):
        """Returns the directional vector of the ROV in the x direction (surge)"""
        x = 1-2*(e2**2+e3**2)
        y = 2*(e1*e2+e3*q0)
        z = 2*(e1*e3-e2*q0)
        return [x, y, z]
    
    def reference_callback(self,msg):
        """Subscriber function for the reference topic"""
        self.mpc1.x_setp = msg.x
        self.mpc1.y_setp = msg.y
        self.mpc1.z_setp = msg.z

        # CALCULATIING THE QUATERNION REFERENCES (Maybe move)
        if(self.ready_signal_mpc):
            if(self.FOV_constraint):
                comparison_vector = [self.mpc1.x_2, self.mpc1.y_2, self.mpc1.z_2]
            else:
                comparison_vector = [float(msg.x), float(msg.y), float(msg.z)]
            this_rov_pos = [float(self.x0[0]), float(self.x0[1]), float(self.x0[2])]
            vector = np.array(comparison_vector) - np.array(this_rov_pos)
            vector = vector / np.linalg.norm(vector)

            theta = np.arccos(np.dot([1, 0, 0], vector))
            axis = np.cross([1, 0, 0], vector)
            if(sum(axis) != 0):
                axis = axis / np.linalg.norm(axis)
                self.mpc1.q_0_setp = np.cos(theta/2)
                self.mpc1.e_1_setp = axis[0] * np.sin(theta/2)
                self.mpc1.e_2_setp = axis[1] * np.sin(theta/2)
                self.mpc1.e_3_setp = axis[2] * np.sin(theta/2)

    def main_odemetry_callback(self, msg):
        """Subscriber function for the main ROV odometry"""
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
        self.x0 = np.array(self.odometry_list)

        if(not self.ready_signal_mpc): #First cycle
            self.mpc1.x0 = self.x0
            self.mpc1.mpc.set_initial_guess()
            self.ready_signal_mpc = True
    
    def odometry_callback_2(self, msg):
        """Subscriber function for 2nd ROV odometry"""
        self.mpc1.x_2 = msg.pose.pose.position.x
        self.mpc1.y_2 = msg.pose.pose.position.y
        self.mpc1.z_2 = msg.pose.pose.position.z

    def odometry_callback_3(self, msg):
        """Subscriber function for 3rd ROV odometry"""
        self.mpc1.x_3 = msg.pose.pose.position.x
        self.mpc1.y_3 = msg.pose.pose.position.y
        self.mpc1.z_3 = msg.pose.pose.position.z
    
    def odometry_callback_4(self, msg):
        """Subscriber function for 4th ROV odometry"""
        self.mpc1.x_4 = msg.pose.pose.position.x
        self.mpc1.y_4 = msg.pose.pose.position.y
        self.mpc1.z_4 = msg.pose.pose.position.z

    def odometry_callback_5(self, msg):
        """Subscriber function for 5th ROV odometry"""
        self.mpc1.x_5 = msg.pose.pose.position.x
        self.mpc1.y_5 = msg.pose.pose.position.y
        self.mpc1.z_5 = msg.pose.pose.position.z
    
    def odometry_callback_6(self, msg):
        """Subscriber function for 6th ROV odometry"""
        self.mpc1.x_6 = msg.pose.pose.position.x
        self.mpc1.y_6 = msg.pose.pose.position.y
        self.mpc1.z_6 = msg.pose.pose.position.z

    def publisher_callback(self):
        """Running the MPC and publishing the thrusts"""
        # Making MPC step
        if(self.ready_signal_mpc): #If the odometry is ready
            self.u0_1 = self.mpc1.mpc.make_step(self.x0)

            # Publishing the thrusts
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
            
            

            self.publisher_1.publish(thrust1)
            self.publisher_2.publish(thrust2)
            self.publisher_3.publish(thrust3)
            self.publisher_4.publish(thrust4)
            self.publisher_5.publish(thrust5)
            self.publisher_6.publish(thrust6)
            self.publisher_7.publish(thrust7)
            self.publisher_8.publish(thrust8)

def main(args=None):
    rclpy.init(args=args)
    bluerov_pubsub_node = BluerovPubSubNode()
    rclpy.spin(bluerov_pubsub_node)
    rclpy.shutdown()
    
main()