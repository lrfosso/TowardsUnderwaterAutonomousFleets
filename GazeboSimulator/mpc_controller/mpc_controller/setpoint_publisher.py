####################### SETPOINT PUBLISHER ###############################

#This node takes in waypoints in as a Vector3 message (no time needed), and creates trajectories 

#example command:
# ros2 topic pub /trajectory_waypoints --once geometry_msgs/msg/Vector3 {'x: 2.0, y: 4.0, z: 2.0'}
import rclpy
from numpy import sin,cos, pi, array, linalg, sqrt
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int32
from geometry_msgs.msg import Vector3

class SetpointPublisher(Node):
    
    def sine_wave(self,t):
        #Creates a sine wave in the z-axis, moves in a circle in the xy-plane 
        z = 1*sin(pi*t/25)+2
        x = 5*cos(t/100)
        y = 5*sin(t/100)
        return x,y,z 

    def torus(self,t):
        #Moves in a spiral shape around a torus of diameter 5
        z = 1*sin(pi*t/25)+2
        x = 5*cos(t/100) + 1*cos(pi*t/25)
        y = 5*sin(t/100) + 1*sin(pi*t/25)
        return x,y,z 
    def line(self,t):
        msg.x = cubic_trajectory_generation(x,t)
        msg.y = cubic_trajectory_generation(y,t)
        msg.z = cubic_trajectory_generation(z,t)
    def cubic_trajectory_parameter_generation(self,x0,dx0,x1,dx1, t0, t1):
        # Returns a vector of coefficients that generate a cubic polynomial that satisfies the constraints
        A =array([[1,t0, t0**2, t0**3],
                    [0, 1, 2*t0, 3*t0**2],
                    [1, t1, t1**2, t1**3],
                    [0, 1, 2*t1, 3*t1**2]])
    
        y = array([x0,dx0,x1,dx1]).reshape(-1,1)
        b = (linalg.inv(A)@y).reshape(1,-1)
        b = b[0]
        print(b)
        return (b[0],b[1],b[2],b[3])
        

    def cubic_trajectory_generation(self,a,t):
        #returns the setpoint value at a given time t with parameters a
        qt = a[0] + a[1]*t + a[2]*t**2 + a[3]*t**3
        return float(qt)

    
    def cubic_trajectory_parameter_generation(self,x0,dx0,x1,dx1, t0, t1):
        # Returns a vector of coefficients that generate a cubic polynomial that satisfies the constraints
        A =array([[1,t0, t0**2, t0**3],
                    [0, 1, 2*t0, 3*t0**2],
                    [1, t1, t1**2, t1**3],
                    [0, 1, 2*t1, 3*t1**2]])
    
        y = array([x0,dx0,x1,dx1]).reshape(-1,1)
        b = (linalg.inv(A)@y).reshape(1,-1)
        b = b[0]
        print(b)
        return (b[0],b[1],b[2],b[3])
        

    def cubic_trajectory_generation(self,a,t):
        #returns the setpoint value at a given time t with parameters a
        qt = a[0] + a[1]*t + a[2]*t**2 + a[3]*t**3
        return float(qt)


    def __init__(self):
        super().__init__('setpoint_publisher')
        self.declare_parameter('debug_rov')

        # Get parameters
        # self. = self.get_parameter('').get_parameter_value().
        self.debug_rov = self.get_parameter('debug_rov').get_parameter_value().integer_value

        self.j = 0 # tracker for which waypoint is active
        self.i = 0 # no. times timer callback has been called
        
        self.control_mode = 2 #control mode initialized to trajectory
        self.std_test = 2 # Standard tests
        self.wp_x = [] #waypoint lists
        self.wp_y = []
        self.wp_z = []
        self.wp_t = [] # time

        self.publisher_ = self.create_publisher(Vector3, 'ref', 10)
        self.a_x = [0]*4 #param list
        self.a_y = [0]*4 #param list
        self.a_z = [0]*4 #param list
        self.timer_period = 0.5  # seconds publish frequency 
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.trajectory_subscriber = self.create_subscription(  # Creates a subscription to the trajectory waypoints topic
            Vector3, #Message type vector of 3 floats
            "/trajectory_waypoints", #Topic
            self.trajectory_callback, #function?
            10)
            
        self.control_subscriber = self.create_subscription(  # Creates a subscription to the trajectory waypoints topic
            Int32, #Integer message type
            "/control_mode", #Topic
            self.control_mode_callback, #function?
            10)
            
        self.std_test_subscriber = self.create_subscription(  # Creates a subscription to the trajectory waypoints topic
            Int32, #Integer message type
            "/std_test", #Topic
            self.std_test_callback, #function?
            10)
    def control_mode_callback(self, msg):
        #self.control_mode = msg.data
        pass

    def std_test_callback(self, msg):
        self.std_test = msg.data

    def trajectory_callback(self, msg): # append waypoints to waypoint list

        avg_speed = 0.4 # average speed in m/s, used to calculate time to each setpoint

        if(len(self.wp_x) == 0): # if first waypoint, append and exit function
            self.wp_x.append(msg.x)
            self.wp_y.append(msg.y)
            self.wp_z.append(msg.z)
            self.wp_t.append(0)
            return

        if (((msg.x == self.wp_x[len(self.wp_x)-1]) and (msg.y == self.wp_y[len(self.wp_y)-1]) and (msg.z == self.wp_z[len(self.wp_z)-1]))): # guard for duplicate waypoints
            return

        if(len(self.wp_t) == 0): # caluclating time needed to  get to each waypoint
            #self.wp_t.append(sqrt(msg.x**2+msg.y**2+msg.z**2)/avg_speed)
            self.wp_t.append(0)
        else: # appends length of vector between the last 2 waypoints divided by the average speed set to find the time needed, + time at previous waypoint
            self.wp_t.append((sqrt((msg.x-self.wp_x[len(self.wp_x)-1])**2 + (msg.y-self.wp_y[len(self.wp_y)-1])**2 + (msg.z-self.wp_z[len(self.wp_z)-1])**2)/avg_speed)+ self.wp_t[len(self.wp_t)-1])

        self.wp_x.append(msg.x)
        self.wp_y.append(msg.y)
        self.wp_z.append(msg.z)

    def timer_callback(self):
        current_time = self.i * self.timer_period #
        msg = Vector3()
        if (self.control_mode == 1):
		    #msg.x,msg.y,msg.z = self.sine_wave(self.i)
            
            print(self.wp_t)
            
            if(len(self.wp_x) > 1): #Skip if waypoint lists are empty
                self.i += 1

                if((self.wp_t[len(self.wp_t)-1]) <= current_time):  # if the last waypoint has been reached, set reference to last waypoint
                    msg.x = self.cubic_trajectory_generation(self.a_x,self.wp_t[self.j]+1) 
                    msg.y = self.cubic_trajectory_generation(self.a_y,self.wp_t[self.j]+1)
                    msg.z = self.cubic_trajectory_generation(self.a_z,self.wp_t[self.j]+1)
                    self.i -= 1 # pauses timer when ROV at last waypoint
                    
                elif ((current_time >= self.wp_t[self.j]) and ((len(self.wp_t)) > (self.j+1))): # if current cubic trajectory has been completed and there are still more waypoints, move to next trajectory spline
                    self.a_x[0], self.a_x[1], self.a_x[2], self.a_x[3] = self.cubic_trajectory_parameter_generation(self.wp_x[self.j],0,self.wp_x[self.j+1],0,current_time,self.wp_t[self.j+1])
                    self.a_y[0], self.a_y[1], self.a_y[2], self.a_y[3] = self.cubic_trajectory_parameter_generation(self.wp_y[self.j],0,self.wp_y[self.j+1],0,current_time,self.wp_t[self.j+1])
                    self.a_z[0], self.a_z[1], self.a_z[2], self.a_z[3] = self.cubic_trajectory_parameter_generation(self.wp_z[self.j],0,self.wp_z[self.j+1],0,current_time,self.wp_t[self.j+1])
		            # set msg and increment j
                    msg.x = self.cubic_trajectory_generation(self.a_x,current_time)
                    msg.y = self.cubic_trajectory_generation(self.a_y,current_time)
                    msg.z = self.cubic_trajectory_generation(self.a_z,current_time)
                    self.j+=1
    
                else: #else, set msg
                    msg.x = self.cubic_trajectory_generation(self.a_x,current_time)
                    msg.y = self.cubic_trajectory_generation(self.a_y,current_time)
                    msg.z = self.cubic_trajectory_generation(self.a_z,current_time)
                
            self.publisher_.publish(msg)

        elif (self.control_mode  == 2):
            self.get_logger().info('hello')
            match self.std_test:
                case 1:
                    msg.x, msg.y, msg.z = self.sine_wave(current_time)
                case 2:
                    msg.x, msg.y, msg.z = self.torus(current_time) 
                #case 3:
                    #avg_speed = 0.3 #m/s
                    #delta_t = sqrt(5**2 + 5**2 + 2**2)/avg_speed
                    #x = cubic_trajectory_parameter_generation(0,0,5,0,t,t + delta_t)
                    #y = cubic_trajectory_parameter_generation(0,0,5,0,t,t + delta_t)
                    #z = cubic_trajectory_parameter_generation(0,0,2,0,t,t + delta_t)
                    #msg.x, msg.y, msg.z = line(current_time)
            self.publisher_.publish(msg)
            self.i += 1
        if self.debug_rov < 2:
            self.get_logger().info('Publishing: x:"%s", y:"%s", z:"%s"' %( msg.x,msg.y,msg.z))
            



def main(args=None):
    rclpy.init(args=args)

    setpoint_publisher = SetpointPublisher()

    rclpy.spin(setpoint_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    setpoint_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
