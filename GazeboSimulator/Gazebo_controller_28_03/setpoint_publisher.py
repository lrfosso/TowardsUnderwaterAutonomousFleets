# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ros2 topic pub /trajectory_waypoints --once geometry_msgs/msg/Vector3 {'x: 2.0, y: 4.0, z: 2.0'}
import rclpy
from numpy import sin,cos, pi, array, linalg, sqrt
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Vector3

class SetpointPublisher(Node):
    def sine_wave(self,t):
        z = 1*sin(pi*t/25)+2
        x = 5*cos(t/100)
        y = 5*sin(t/100)
        return x,y,z 
    
    def cubic_trajectory_parameter_generation(self,x0,dx0,x1,dx1, t0, t1):
    
        A =array([[1,t0, t0**2, t0**3],
                    [0, 1, 2*t0, 3*t0**2],
                    [1, t1, t1**2, t1**3],
                    [0, 1, 2*t1, 3*t1**2]])
    
        #b = np.array = ([a0,a1,a2,a3]).reshape(-1,1)
        y = array([x0,dx0,x1,dx1]).reshape(-1,1)
        b = (linalg.inv(A)@y).reshape(1,-1)
        b = b[0]
        print(b)
        return (b[0],b[1],b[2],b[3])
        

    def cubic_trajectory_generation(self,a,t):
        qt = a[0] + a[1]*t + a[2]*t**2 + a[3]*t**3
        return float(qt)


    def __init__(self):
        super().__init__('setpoint_publisher')
        self.j = 0 # tracker for which waypoint is active
        self.i = 0 # no. times timer callback has been called
        #self.wp_x = [0,10, 5, 10, 0] #waypoint lists
        #self.wp_y = [0,10, 5, 10, 0]
        #self.wp_z = [2, 2, 2, 2, 2]
        #self.wp_t = [0,60, 100, 150, 300] # time

        self.wp_x = [] #waypoint lists
        self.wp_y = []
        self.wp_z = []
        self.wp_t = [] # time

        self.publisher_ = self.create_publisher(Vector3, 'ref', 10)
        self.a_x = [0]*4 #param list
        self.a_y = [0]*4 #param list
        self.a_z = [0]*4 #param list
        #self.a_x[0], self.a_x[1], self.a_x[2], self.a_x[3] = self.cubic_trajectory_parameter_generation(0,0,20,0,self.i,self.i+150)
        #self.a_y[0], self.a_y[1], self.a_y[2], self.a_y[3] = self.cubic_trajectory_parameter_generation(0,0,15,0,self.i,self.i+150)
        #self.a_z[0], self.a_z[1], self.a_z[2], self.a_z[3] = self.cubic_trajectory_parameter_generation(2,0,3,0,self.i,self.i+150)
        self.timer_period = 0.5  # seconds publish frequency 
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.trajectory_subscriber = self.create_subscription(  # Creates a subscription to the trajectory waypoints topic
            Vector3, #Message type vector of 3 floats
            "/trajectory_waypoints", #Topic
            self.trajectory_callback, #function?
            10)

    def trajectory_callback(self, msg): # append waypoints to waypoint list

        avg_speed = 0.3 # average speed i m/s

        if(len(self.wp_x) == 0): # if first waypoint, append and exit function
            self.wp_x.append(msg.x)
            self.wp_y.append(msg.y)
            self.wp_z.append(msg.z)
            self.wp_t.append(0)
            print(msg.x)
            return

        if (((msg.x == self.wp_x[len(self.wp_x)-1]) and (msg.y == self.wp_y[len(self.wp_y)-1]) and (msg.z == self.wp_z[len(self.wp_z)-1]))): # guard for duplicate waypoints
            return

        if(len(self.wp_t) == 0): # caluclating time needed to  get to each waypoint
            #self.wp_t.append(sqrt(msg.x**2+msg.y**2+msg.z**2)/avg_speed)
            self.wp_t.append(0)
        else: # appends length of vector between the last 2 waypointd devided by the average speed set to find the time needed, + time at previous waypoint
            self.wp_t.append((sqrt((msg.x-self.wp_x[len(self.wp_x)-1])**2 + (msg.y-self.wp_y[len(self.wp_y)-1])**2 + (msg.z-self.wp_z[len(self.wp_z)-1])**2)/avg_speed)+ self.wp_t[len(self.wp_t)-1])

        self.wp_x.append(msg.x)
        self.wp_y.append(msg.y)
        self.wp_z.append(msg.z)
        print(self.wp_x)

    def timer_callback(self):
        current_time = self.i * self.timer_period
        msg = Vector3()
        #msg.x,msg.y,msg.z = self.sine_wave(self.i)
        #if (self. i == 150):
        #    self.a_x[0], self.a_x[1], self.a_x[2], self.a_x[3] = self.cubic_trajectory_parameter_generation(20,0,0,0,self.i,self.i+150)
        #    self.a_y[0], self.a_y[1], self.a_y[2], self.a_y[3] = self.cubic_trajectory_parameter_generation(15,0,0,0,self.i,self.i+150)
        #    self.a_z[0], self.a_z[1], self.a_z[2], self.a_z[3] = self.cubic_trajectory_parameter_generation(3,0,2,0,self.i,self.i+150)

        #if( self.i < 150):
        #    msg.x = self.cubic_trajectory_generation(self.a_x,self.i)
        #    msg.y = self.cubic_trajectory_generation(self.a_y,self.i)
        #    msg.z = self.cubic_trajectory_generation(self.a_z,self.i)
        #    
        #elif(self.i < 300):
        #    msg.x = self.cubic_trajectory_generation(self.a_x,self.i)
        #    msg.y = self.cubic_trajectory_generation(self.a_y,self.i)
        #    msg.z = self.cubic_trajectory_generation(self.a_z,self.i)



        #self.get_logger().info('i = ' + str(self.i))
        #self.get_logger().info('j = ' + str(self.j))
        #self.get_logger().info('w = ' + str(self.wp_t[self.j]))
        #self.get_logger().info('w = ' + str(len(self.wp_t)))
        print(self.wp_t)
        if(len(self.wp_x) > 1):
            self.i += 1
            if((self.wp_t[len(self.wp_t)-1]) <= current_time):  # if the last waypoint has been reached, set reference to last waypoint
                print("1")
                msg.x = self.cubic_trajectory_generation(self.a_x,self.wp_t[self.j]+1) 
                msg.y = self.cubic_trajectory_generation(self.a_y,self.wp_t[self.j]+1)
                msg.z = self.cubic_trajectory_generation(self.a_z,self.wp_t[self.j]+1)
                self.i -= 1 # pauses timer when ROV at last waypoint
            
            elif ((current_time >= self.wp_t[self.j]) and ((len(self.wp_t)) > (self.j+1))): # if current cubic trajectory has been completed and there are still more waypoints, move to next trajectory 
                self.a_x[0], self.a_x[1], self.a_x[2], self.a_x[3] = self.cubic_trajectory_parameter_generation(self.wp_x[self.j],0,self.wp_x[self.j+1],0,current_time,self.wp_t[self.j+1])
                self.a_y[0], self.a_y[1], self.a_y[2], self.a_y[3] = self.cubic_trajectory_parameter_generation(self.wp_y[self.j],0,self.wp_y[self.j+1],0,current_time,self.wp_t[self.j+1])
                self.a_z[0], self.a_z[1], self.a_z[2], self.a_z[3] = self.cubic_trajectory_parameter_generation(self.wp_z[self.j],0,self.wp_z[self.j+1],0,current_time,self.wp_t[self.j+1])
    
                msg.x = self.cubic_trajectory_generation(self.a_x,current_time)
                msg.y = self.cubic_trajectory_generation(self.a_y,current_time)
                msg.z = self.cubic_trajectory_generation(self.a_z,current_time)
                print("hello")    
                self.j+=1
    
            else:
                print("3")
                msg.x = self.cubic_trajectory_generation(self.a_x,current_time)
                msg.y = self.cubic_trajectory_generation(self.a_y,current_time)
                msg.z = self.cubic_trajectory_generation(self.a_z,current_time)
    
    
        self.publisher_.publish(msg)
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
