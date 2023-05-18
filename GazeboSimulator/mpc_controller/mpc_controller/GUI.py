"""_________________________________________________________________________________________________
ROS 2 IMPLEMENTED GUI FOR A ROV SIMULATOR IN GAZEBO
____________________________________________________________________________________________________
The GUI is made with PySimpleGUI and is used to control the ROVs in Gazebo.
Possibility to run 1-3 agents.
Functionality:
    - Displaying current states and status of the ROVs
    - Set waypoints for the ROV
    - Start and stop recording of data
    - Run standard tests on the ROV
    - Set ocean current
    - Control the ROV with a joystick
    - Read system parameters
_________________________________________________________________________________________________"""

#Import Python modules and functions
import rclpy
from rclpy.node import Node
import PySimpleGUI as sg
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import os
import numpy as np
import time

# Import ROS2 libraries and tools
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from std_msgs.msg import String

class GUI(Node):

    def __init__(self):
        """INIT ROS2 GUI NODE"""
        ###### INIT ROS2 ############################################################################
        super().__init__('GUI')
        ###### INIT ROS2 PARAMETERS
        self.declare_parameter('n_multi_agent')
        self.declare_parameter('FOV_max')
        self.n_multi_agent = self.get_parameter('n_multi_agent').get_parameter_value().integer_value
        self.FOV_limit = self.get_parameter('FOV_max').get_parameter_value().double_value
        ###### INIT SUBSCRIBERS AND PUBLISHERS
        self.subscription = self.create_subscription(
            Odometry,
            '/bluerov2_pid/bluerov2/observer/nlo/odom_ned',
            self.ROV1_main_callback,
            10)
        self.subscription = self.create_subscription(
                Vector3,
                '/ref',
                self.ref_callback,
                10)
        self.subscription = self.create_subscription(
                Bool,
                '/ready_next_stdtest',
                self.std_test_sub_callback,
                10)
        if(self.n_multi_agent > 1):
            self.subscription = self.create_subscription(
                Odometry,
                '/bluerov2_pid/bluerov3/observer/nlo/odom_ned',
                self.ROV2_odom_callback,
                10)
            self.subscription = self.create_subscription(
                Float64,
                '/bluerov2_mpc/angle/from_2_to_3',
                self.angle_callback2to3,
                10)
            self.subscription = self.create_subscription(
                Float64,
                '/bluerov3_mpc/angle/from_3_to_2',
                self.angle_callback3to2,
                10)
        if(self.n_multi_agent > 2):
            self.subscription = self.create_subscription(
                Odometry,
                '/bluerov2_pid/bluerov4/observer/nlo/odom_ned',
                self.ROV3_odom_callback,
                10)
            self.subscription = self.create_subscription(
                Float64,
                '/bluerov2_mpc/angle/from_2_to_4',
                self.angle_callback2to4,
                10)
            self.subscription = self.create_subscription(
                Float64,
                '/bluerov3_mpc/angle/from_3_to_4',
                self.angle_callback3to4,
                10)
            self.subscription = self.create_subscription(
                Float64,
                '/bluerov4_mpc/angle/from_4_to_2',
                self.angle_callback4to2,
                10)
            self.subscription = self.create_subscription(
                Float64,
                '/bluerov4_mpc/angle/from_4_to_3',
                self.angle_callback4to3,
                10)

        self.subscription  # prevent unused variable warning
        self.publisher_1 = self.create_publisher(Vector3, '/trajectory_waypoints', 10)
        self.publisher_control_mode = self.create_publisher(Int32, '/control_mode', 10)
        self.publisher_standard_test = self.create_publisher(Int32, "/std_test", 10)
        self.publisher_record = self.create_publisher(Bool, "/record_data", 10)
        self.publisher_filename = self.create_publisher(String, "/filename_data", 10)

        ###### INIT PLOT ############################################################################
        w, h = figsize = (10, 7)     # figure size
        self.fig, self.ax = plt.subplots(figsize=figsize)
        dpi = self.fig.get_dpi()
        self.size = (int(w*dpi), int(h*dpi))
        self.ax.grid(True)
        self.ax.set(xlim=(-10, 10), ylim=(-10, 10))
        self.ax.set_title("Bird's-eye view", fontdict={'fontsize': 20})
        self.ax.set_xlabel('y', fontdict={'fontsize': 15})
        self.ax.set_ylabel('x', fontdict={'fontsize': 15})
        self.ax.set_facecolor("blue")

        ###### INIT PYSIMPLEGUI ######################################################################
        self.setup_layout()
        self.window = sg.Window('ROV Simulator GUI', self.layout, finalize=True,size=(1800, 900), element_justification='center')
        ## Changing the color of the cursor in the input boxes
        self.window['-FILENAME-'].Widget.config(insertbackground='black')
        self.window['-X-'].Widget.config(insertbackground='black')
        self.window['-Y-'].Widget.config(insertbackground='black')
        self.window['-Z-'].Widget.config(insertbackground='black')
        ## Setting up canvas
        self.canvas_elem = self.window['-CANVAS-']
        self.canvas = self.canvas_elem.TKCanvas
        self.fig_agg = self.draw_figure(self.canvas, self.fig)

        ## Initializing variables to be used in the main loop
        self.traj_reference = [0,0,0]
        self.pos2 = [0,0,0]
        self.pos3 = [0,0,0]

        self.trajectory_log_1 = [[],[]]
        self.trajectory_log_2 = [[],[]]
        self.trajectory_log_3 = [[],[]]

        self.final_dest_x = 0
        self.final_dest_y = 0

        self.std_test = Int32()
        self.std_test.data = 0

        self.record = Bool()
        self.record.data = False

        self.std_test_ready_next = True
        self.cooldown_start_std_test = False
        self.standard_test_num = 2
        self.init_next_test = True
        self.sequence_test = False
        self.start_cooldown = time.time()
        self.wave_direction = True
        self.waves_active = False
        self.std_test_nr = 84

        # Ensuring that the ocean current is set to zero
        os.system("gz topic -t /ocean_current -m gz.msgs.Vector3d -p 'x: 0, y:0, z:0'")

    def ROV1_main_callback(self, msg):
        """Callback from main odometry and main PySimpleGUI loop"""
        ### get the position of the ROV
        self.odom1 = msg.pose.pose.position
        event, self.values = self.window.read(timeout=5)
        if event in ('-EXIT-', None): # if user closes window or clicks cancel. Stop the program
            exit(69)
        if event == '-SET_P-':      # Setting new waypoint
            self.publish_waypoint()
        if event == '-READ-':       # Reading parameters from file
            self.open_window()
        if event == '-SET_CUR-':    # Setting ocean current
            self.set_ocean_current()
        if event == '-RESET_CUR-': # Resetting ocean current
            os.system("gz topic -t /ocean_current -m gz.msgs.Vector3d -p 'x: 0, y:0, z:0'")
        
        # RUN STANDARD TESTS(1-4) OR STANDARD TEST SEQUENCE
        if event == '-STD_TEST_1-':
            self.std_test.data = 1
            self.publisher_standard_test.publish(self.std_test)
            self.sequence_test = False
        elif event == '-STD_TEST_2-':
            self.std_test.data = 2
            self.publisher_standard_test.publish(self.std_test)
            self.sequence_test = False
        elif event == '-STD_TEST_3-':
            self.std_test.data = 3
            self.publisher_standard_test.publish(self.std_test)
            self.sequence_test = False
        elif event == '-STD_TEST_4-':
            self.std_test.data = 4
            self.publisher_standard_test.publish(self.std_test)
            self.sequence_test = False
        elif event == '-INITIALIZE-':
            self.std_test.data = 0
            self.publisher_standard_test.publish(self.std_test)
            self.sequence_test = False
        elif event == '-STD_TEST_SEQUENCE-':
            self.sequence_test = True
            self.std_test_ready_next = True
            self.cooldown_start_std_test = False
            self.init_next_test = False

        ## RUNNING WAVES
        #if(self.sequence_test and self.waves_active):
        #    wave_size = 0.5
        #    wave_timer = time.time()
        #    if(wave_timer > self.wave_period_timer + 1):
        #        self.wave_direction = not self.wave_direction
        #        os.system("gz topic -t /ocean_current -m gz.msgs.Vector3d -p 'x: {}, y:0, z:{}'".format(wave_size ,wave_size if self.wave_direction else -wave_size))
        #        self.get_logger().info("Switching waves")
        #        self.wave_period_timer = time.time()
        #else:
        #    self.wave_period_timer = time.time()

        # SETTING UP AND RUNNING STANDARD TEST SEQUENCE
        test_name = "circle_setp_{}_".format(self.std_test_nr)
        standard_test = [test_name+"circle", test_name+"torus", test_name+"line", test_name+"spiral"]

        if (self.std_test_ready_next and self.sequence_test):
             # Setting up the next test
            if(self.init_next_test):
                if(self.standard_test_num == 4):
                    self.standard_test_num = 1
                    self.std_test_nr += 1
                else:
                    self.standard_test_num += 1
                self.init_next_test = False
                self.waves_active = False
            test_name = "circle_setp_{}_".format(self.std_test_nr)
            standard_test = [test_name+"circle", test_name+"torus", test_name+"line", test_name+"spiral"]
            self.filename = String()
            self.filename.data = standard_test[self.standard_test_num - 1]
            self.window['-FILENAME-'].update(self.filename.data)
            self.publisher_filename.publish(self.filename)
            self.record.data = False
            self.publisher_record.publish(self.record)
            self.window['-RECORD-'].update(text="Start")
            #os.system("gz topic -t /ocean_current -m gz.msgs.Vector3d -p 'x: 0, y:0, z:0'") # USED WHEN RUNNING WAVES

            cooldown_timer = time.time()
            length1 = np.sqrt((self.odom1.x)**2+(self.odom1.y)**2+(5-self.odom1.z)**2)
            length2 = np.sqrt((self.pos2[0])**2+(self.pos2[1])**2+(5-self.pos2[2])**2)
            # Checking if ROVs are in position for next test
            if(length1 < 2 and
                length2 < 2 and
                self.angle3to2 < 15 and
                self.angle2to3 < 15):
                # Starting the next test
                if(not self.cooldown_start_std_test):
                    self.start_cooldown = time.time()
                    self.cooldown_start_std_test = True
                if(cooldown_timer > self.start_cooldown + 5):
                    self.get_logger().info("Ready go next")
                    self.record.data = True
                    self.publisher_record.publish(self.record)
                    self.window['-RECORD-'].update(text="Stop")

                    self.std_test.data = self.standard_test_num
                    self.publisher_standard_test.publish(self.std_test)

                    self.cooldown_start_std_test = False
                    self.waves_active = True
                    self.wave_direction = True
                    
                    self.wave_period_timer = time.time()
                    self.std_test_ready_next = False
                    self.init_next_test = True
                    
            else:
                self.start_cooldown = time.time()
                self.cooldown_start_std_test = False
            

                
        # Colors for the control mode buttons
        useable_col = ('black',"Grey80")
        unuseable_col = ('black',"Blue4")
        mode = Int32()
        # Choosing control mode
        if True == self.values['-JOYSTICK-']:
            mode.data = 0
            self.JOY_mode(useable_col, unuseable_col)
        elif True == self.values['-TRAJECTORY-']:
            mode.data = 1
            self.TRAJECTORY_mode(useable_col, unuseable_col)
        else:
            mode.data = 2
            self.STANDARD_TEST_mode(useable_col, unuseable_col)
        self.publisher_control_mode.publish(mode)

        if event == '-RECORD-':    # Start and stop recording data
            self.record.data = not self.record.data
            if self.record.data:
                self.window['-RECORD-'].update(text="Stop")
            else:
                self.window['-RECORD-'].update(text="Start")
        self.publisher_record.publish(self.record)
        self.filename = String()
        self.filename.data = self.values['-FILENAME-']
        self.publisher_filename.publish(self.filename)

        
        self.reinitialize_plot()

        # Plot the reference and final destination
        self.ax.scatter(self.traj_reference[1], self.traj_reference[0], c='black', s=40, label='Reference', marker='x')
        self.ax.scatter(self.final_dest_y,self.final_dest_x ,c='black', s=40, label='Final dest.')

        # Update the canvas with ROV1
        if self.values['-TRAJ1-'] == True:
            if(len(self.trajectory_log_1[0]) > 2000 and len(self.trajectory_log_1[0]) > 0):
                self.trajectory_log_1[0].pop(0)
                self.trajectory_log_1[1].pop(0)
            self.trajectory_log_1[0].append(self.odom1.x)
            self.trajectory_log_1[1].append(self.odom1.y)
        else:
            self.trajectory_log_1 = [[],[]]
        self.ax.scatter(self.odom1.y, self.odom1.x, c='blue', s=40, label='ROV 2')
        self.ax.plot(self.trajectory_log_1[1], self.trajectory_log_1[0], c='blue')
        # Update the canvas with ROV2
        if self.n_multi_agent > 1:
            if self.values['-TRAJ2-'] == True:
                if(len(self.trajectory_log_2[0]) > 2000 and len(self.trajectory_log_2[0]) > 0):
                    self.trajectory_log_2[0].pop(0)
                    self.trajectory_log_2[1].pop(0)
                self.trajectory_log_2[0].append(self.pos2[0])
                self.trajectory_log_2[1].append(self.pos2[1])
            else:
                self.trajectory_log_2 = [[],[]]
            self.ax.scatter(self.pos2[1], self.pos2[0], c='green', s=40, label='ROV 3')
            self.ax.plot(self.trajectory_log_2[1], self.trajectory_log_2[0], c='green')
        # Update the canvas with ROV3
        if self.n_multi_agent > 2:
            if self.values['-TRAJ3-'] == True:
                if(len(self.trajectory_log_3[0]) > 2000 and len(self.trajectory_log_3[0]) > 0):
                    self.trajectory_log_3[0].pop(0)
                    self.trajectory_log_3[1].pop(0)
                self.trajectory_log_3[0].append(self.pos3[0])
                self.trajectory_log_3[1].append(self.pos3[1])
            else:
                self.trajectory_log_3 = [[],[]]
            self.ax.scatter(self.pos3[1], self.pos3[0], c='red', s=40, label='ROV 4')
            self.ax.plot(self.trajectory_log_3[1], self.trajectory_log_3[0], c='red')
        # Update the canvas with the new plot
        self.update_xyz_GUI_indication()
        self.ax.legend()
        self.fig_agg.draw()

    def update_xyz_GUI_indication(self):
        """Update the xyz values in the GUI"""
        self.window['-X_visual-'].update("%.2f"%self.odom1.x)
        self.window['-Y_visual-'].update("%.2f"%self.odom1.y)
        self.window['-Z_visual-'].update("%.2f"%self.odom1.z)
        if(self.n_multi_agent > 1):
            self.window['-X_visual2-'].update("%.2f"%self.pos2[0])
            self.window['-Y_visual2-'].update("%.2f"%self.pos2[1])
            self.window['-Z_visual2-'].update("%.2f"%self.pos2[2])
        if(self.n_multi_agent > 2):
            self.window['-X_visual3-'].update("%.2f"%self.pos3[0])
            self.window['-Y_visual3-'].update("%.2f"%self.pos3[1])
            self.window['-Z_visual3-'].update("%.2f"%self.pos3[2])

    def reinitialize_plot(self):
        """Setup the plot. Runs every iteration of the main loop"""
        self.ax.cla()
        self.ax.grid(True)
        self.ax.set(xlim=(-20, 20), ylim=(-20, 20))
        self.ax.set_title("Bird's-eye view", fontdict={'fontsize': 20})
        self.ax.set_xlabel('y', fontdict={'fontsize': 15})
        self.ax.set_ylabel('x', fontdict={'fontsize': 15})
        self.ax.set_facecolor((0.1,0.9,1))

    def setup_layout(self):
        """"Setup the layout of the GUI"""
        background_col = "RoyalBlue4"
        text_col = "Grey93"
        clickable_backgr_col = "Grey93"
        clickable_text_col = "black"

        unclickable_col = "Blue4"
        button_col = "Grey80"
        ind_text_col = '#0000B9'
        ROV_col_width = 17
        checkbox_spacing_w = 5

        font = 'Ubuntu'

        self.first_col = [
            [sg.Text('ROV Simulator Settings', size=(28, 1), justification='center', font=(font, 25, "bold"),text_color=text_col, background_color=unclickable_col)],
            [sg.Text('', background_color=background_col)],
            [sg.Text('Record rov data as CSV', size=(55, 1), justification='center', font=font,text_color=text_col, background_color=unclickable_col)],
            [sg.Text('Filename', size=(7, 1), justification='center', font=font,text_color=text_col, background_color=unclickable_col),
            sg.InputText(default_text="rov",key='-FILENAME-', size=(30, 1), font=font, background_color=clickable_backgr_col, text_color=clickable_text_col),
            sg.Button('Start', key='-RECORD-', size=(10, 1), font=font, button_color=("black", button_col)),
            ],
            [sg.Text('Control mode', size=(55, 1), justification='center', font=font,text_color=text_col, background_color=unclickable_col),],
            [sg.Radio('Manual (Joystick)               ', "Control_mode", key='-JOYSTICK-', default=False,text_color="black", font=font, background_color=button_col, size=(50, 1))],
            [sg.Radio('Autonomous (Trajectory planning)', "Control_mode", key='-TRAJECTORY-', text_color="black", font=font, background_color=button_col, default=False, size=(50, 1))],
            [sg.Radio('Standard test                   ', "Control_mode", key='-STANDARD_TEST-',text_color="black", font=font, background_color=button_col, default=True, size=(50, 1))],
            [sg.Text('', background_color=background_col)],
            [sg.Text('',font=font, size=(5, 1),text_color=text_col, background_color=background_col), 
            sg.Text('Position waypoint', size=(22, 1), justification='center', font=(font, 12, "bold"),text_color=text_col, background_color=unclickable_col),
            sg.Text('Ocean current', size=(22, 1), justification='center', font=font,text_color=text_col, background_color=unclickable_col)],
            [sg.Text('X:',font=font, size=(4, 1),text_color=text_col, background_color=unclickable_col), 
            sg.InputText(size=(22, 1), pad=((10, 0), 3), font=font, key='-X-',text_color=clickable_text_col, background_color=clickable_backgr_col, default_text='0'),
            #sg.Text('', size=(1,1), background_color=background_col),
            sg.InputText('0', size=(22, 1), justification='center', font=font, key='-CUR_X-',text_color=clickable_text_col, background_color=clickable_backgr_col),
            ],
            [sg.Text('Y:',font=font, size=(4, 1),text_color=text_col, background_color=unclickable_col), 
            sg.InputText(size=(22, 1), pad=((10, 0), 3), font=font, key='-Y-',text_color=clickable_text_col, background_color=clickable_backgr_col, default_text='0'),
            sg.InputText('0', size=(22, 1), justification='center', font=font, key='-CUR_Y-',text_color=clickable_text_col, background_color=clickable_backgr_col),
            ],
            [sg.Text('Z:',font=font, size=(4, 1),text_color=text_col, background_color=unclickable_col), 
            sg.InputText(size=(22, 1), pad=((10, 0), 3), font=font, key='-Z-',text_color=clickable_text_col, background_color=clickable_backgr_col, default_text='0'),
            sg.InputText('0', size=(22, 1), justification='center', font=font, key='-CUR_Z-',text_color=clickable_text_col, background_color=clickable_backgr_col),
            ],
            [sg.Text('',font=font, size=(5, 1),text_color=text_col, background_color=background_col), 
            sg.Button('Set position', size=(19, 1), font=font, key='-SET_P-', button_color=('black',button_col)),
            sg.Button('Set current', size=(19, 1), font=font, key='-SET_CUR-', button_color=('black', button_col)),
            ],
            [sg.Text('', size=(36,1),background_color=background_col),
            sg.Button('Reset ocean current', size=(19, 1), font=font, key='-RESET_CUR-', button_color=('black', button_col)),
            ],
            [sg.Text('', background_color=background_col)],
            [sg.Text('Standard test', size=(50, 1), justification='center', font=(font, 12, "bold"),text_color=text_col, background_color=unclickable_col)],
            [sg.Button('Circle [1]', size=(20, 1), font=font, key='-STD_TEST_1-', button_color=('black', button_col)),
            sg.Button('Torus [2]', size=(20, 1), font=font, key='-STD_TEST_2-', button_color=('black', button_col))],
            [sg.Button('Line [3]', size=(20, 1), font=font, key='-STD_TEST_3-', button_color=('black', button_col)),
            sg.Button('Spiral [4]', size=(20, 1), font=font, key='-STD_TEST_4-', button_color=('black', button_col))],
            [sg.Button('Initialize position', size=(20, 1), font=font, key='-INITIALIZE-', button_color=('black', button_col)),
            sg.Button('Test sequence', size=(20, 1), font=font, key='-STD_TEST_SEQUENCE-', button_color=('black', button_col))],
            [sg.Text('', background_color=background_col)],
            [sg.Text('O', background_color=background_col, size=(5, 1), text_color=background_col, justification='center', font=font),
            sg.Text('ROV2', size=(ROV_col_width, 1), justification='center', font=font,text_color=text_col, background_color=unclickable_col),
            sg.Text('ROV3', size=(ROV_col_width, 1), justification='center', font=font,text_color=text_col, background_color=unclickable_col) if self.n_multi_agent > 1 else sg.Text('',size=(0,0), background_color=background_col),
            sg.Text('ROV4', size=(ROV_col_width, 1), justification='center', font=font,text_color=text_col, background_color=unclickable_col) if self.n_multi_agent > 2 else sg.Text('',size=(0,0), background_color=background_col),
            ],
            [sg.Text('X', size=(5, 1), justification='center', font=font,text_color=text_col, background_color=unclickable_col),
            sg.Text('X', size=(ROV_col_width, 1), justification='center', font=font, key='-X_visual-',text_color=text_col, background_color=ind_text_col),
            sg.Text('X', size=(ROV_col_width, 1), justification='center', font=font, key='-X_visual2-',text_color=text_col, background_color=ind_text_col) if self.n_multi_agent > 1 else sg.Text('',size=(0,0), background_color=background_col),
            sg.Text('X', size=(ROV_col_width, 1), justification='center', font=font, key='-X_visual3-',text_color=text_col, background_color=ind_text_col) if self.n_multi_agent > 2 else sg.Text('',size=(0,0), background_color=background_col),
            
            ],
            [sg.Text('Y', size=(5, 1), justification='center', font=font,text_color=text_col, background_color=unclickable_col),
            sg.Text('Y', size=(ROV_col_width, 1), justification='center', font=font, key='-Y_visual-',text_color=text_col, background_color=ind_text_col),
            sg.Text('Y', size=(ROV_col_width, 1), justification='center', font=font, key='-Y_visual2-',text_color=text_col, background_color=ind_text_col) if self.n_multi_agent > 1 else sg.Text('',size=(0,0), background_color=background_col),
            sg.Text('Y', size=(ROV_col_width, 1), justification='center', font=font, key='-Y_visual3-',text_color=text_col, background_color=ind_text_col) if self.n_multi_agent > 2 else sg.Text('',size=(0,0), background_color=background_col),
            ],
            [sg.Text('Z', size=(5, 1), justification='center', font=font,text_color=text_col, background_color=unclickable_col),
            sg.Text('Z', size=(ROV_col_width, 1), justification='center', font=font, key='-Z_visual-',text_color=text_col, background_color=ind_text_col),
            sg.Text('Z', size=(ROV_col_width, 1), justification='center', font=font, key='-Z_visual2-',text_color=text_col, background_color=ind_text_col) if self.n_multi_agent > 1 else sg.Text('',size=(0,0), background_color=background_col),
            sg.Text('Z', size=(ROV_col_width, 1), justification='center', font=font, key='-Z_visual3-',text_color=text_col, background_color=ind_text_col) if self.n_multi_agent > 2 else sg.Text('',size=(0,0), background_color=background_col),
            ],
            [sg.Text('Plot\nPath', size=(5, 2), justification='center', font=font,text_color=text_col, background_color=unclickable_col),
            sg.Text('', size=(checkbox_spacing_w, 2), background_color=background_col, font=font,text_color=text_col),
            sg.Checkbox('', default=False, key='-TRAJ1-',text_color="black", font=font, background_color=button_col),
            sg.Text('', size=(checkbox_spacing_w-1, 2), background_color=background_col, font=font,text_color=text_col) if self.n_multi_agent > 1 else sg.Text('',size=(0,0), background_color=background_col),
            sg.Text('', size=(checkbox_spacing_w+2, 2), background_color=background_col, font=font,text_color=text_col) if self.n_multi_agent > 1 else sg.Text('',size=(0,0), background_color=background_col),
            sg.Checkbox('', default=False, key='-TRAJ2-',text_color="black", font=font, background_color=button_col) if self.n_multi_agent > 1 else sg.Text('',size=(0,0), background_color=background_col),
            sg.Text('', size=(checkbox_spacing_w-2, 2), background_color=background_col, font=font,text_color=text_col) if self.n_multi_agent > 1 else sg.Text('',size=(0,0), background_color=background_col),
            sg.Text('', size=(checkbox_spacing_w+3, 2), background_color=background_col, font=font,text_color=text_col) if self.n_multi_agent > 2 else sg.Text('',size=(0,0), background_color=background_col),
            sg.Checkbox('', default=False, key='-TRAJ3-',text_color="black", font=font, background_color=button_col) if self.n_multi_agent > 2 else sg.Text('',size=(0,0), background_color=background_col),  
            sg.Text('', size=(checkbox_spacing_w+1, 2), background_color=background_col, font=font,text_color=text_col) if self.n_multi_agent > 2 else sg.Text('',size=(0,0), background_color=background_col),    
            ],
            [sg.Button('Read System Parameters', size=(9, 2), font=font, key='-READ-', button_color=('black', button_col), pad=((0, 0), (10, 0))),
            sg.Text('', background_color=background_col, size=(20, 1)),
            sg.Button('Exit', size=(25, 2), font=font, key='-EXIT-', button_color=('white', 'red'), pad=((0, 0), (10, 0)))
            ],
            ]
        self.sec_col = [
            [sg.Canvas(size=self.size, key='-CANVAS-', background_color='white')],
            [
            sg.Text('FOV:', size=(15, 1), justification='center', font=font,text_color=text_col, background_color=unclickable_col),
            ],
            [
            sg.Text('2 to 3:', size=(7, 1), justification='center', font=font,text_color=text_col, background_color=unclickable_col) if self.n_multi_agent > 1 else sg.Text('',size=(0,0), background_color="white"),
            sg.Text('??',size=(5, 1), justification='center', key='-ANGLE_23-',font=font,text_color=text_col, background_color=ind_text_col) if self.n_multi_agent > 1 else sg.Text('',size=(0,0), background_color="white"),
            sg.Text('3 to 2:', size=(7, 1), justification='center', font=font,text_color=text_col, background_color=unclickable_col) if self.n_multi_agent > 1 else sg.Text('',size=(0,0), background_color="white"),
            sg.Text('??',size=(5, 1), justification='center', key='-ANGLE_32-',font=font,text_color=text_col, background_color=ind_text_col) if self.n_multi_agent > 1 else sg.Text('',size=(0,0), background_color="white"),
            sg.Text('4 to 2:', size=(7, 1), justification='center', font=font,text_color=text_col, background_color=unclickable_col) if self.n_multi_agent > 2 else sg.Text('',size=(0,0), background_color="white"),
            sg.Text('??',size=(5, 1), justification='center', key='-ANGLE_42-',font=font,text_color=text_col, background_color=ind_text_col) if self.n_multi_agent > 2 else sg.Text('',size=(0,0), background_color="white"),
            ],
            [
            sg.Text('2 to 4:', size=(7, 1), justification='center', font=font,text_color=text_col, background_color=unclickable_col) if self.n_multi_agent > 2 else sg.Text('',size=(0,0), background_color="white"),
            sg.Text('??',size=(5, 1), justification='center', key='-ANGLE_24-',font=font,text_color=text_col, background_color=ind_text_col) if self.n_multi_agent > 2 else sg.Text('',size=(0,0), background_color="white"),
            sg.Text('3 to 4:', size=(7, 1), justification='center', font=font,text_color=text_col, background_color=unclickable_col) if self.n_multi_agent > 2 else sg.Text('',size=(0,0), background_color="white"),
            sg.Text('??',size=(5, 1), justification='center', key='-ANGLE_34-',font=font,text_color=text_col, background_color=ind_text_col) if self.n_multi_agent > 2 else sg.Text('',size=(0,0), background_color="white"),
            sg.Text('4 to 3:', size=(7, 1), justification='center', font=font,text_color=text_col, background_color=unclickable_col) if self.n_multi_agent > 2 else sg.Text('',size=(0,0), background_color="white"),
            sg.Text('??',size=(5, 1), justification='center', key='-ANGLE_43-',font=font,text_color=text_col, background_color=ind_text_col) if self.n_multi_agent > 2 else sg.Text('',size=(0,0), background_color="white"),
            ],
            
        ]
        sg.theme('DarkTanBlue')
        self.layout = [
            [sg.Column(self.first_col,background_color=background_col, element_justification='center'),
             sg.VSeperator(),
             sg.Column(self.sec_col, background_color='white'),]
        ]

    def publish_waypoint(self):
        """Publish waypoint to the topic"""
        ## Checking if the waypoint coordinates are valid
        invalid = False
        try:
            float(self.values['-X-'])
            float(self.values['-Y-'])
            float(self.values['-Z-'])
            if(float(self.values['-X-']) > 20 or float(self.values['-X-']) < -20
                or float(self.values['-Y-']) > 20 or float(self.values['-Y-']) < -20
                or float(self.values['-Z-']) > 15 or float(self.values['-Z-']) < 0):
                invalid = True
        except ValueError:
            invalid = True
        ## Settnig the limits for the waypoint coordinates

        if(invalid):
            sg.popup('Please enter a valid number for the waypoint coordinates!\n\nLimits:\nX: Min: -20\tMax: 20\nY: Min: -20\tMax: 20\n  Z: Min: 0\tMax: 15',background_color="yellow", text_color="black", font="Helvetica 14", title="Warning")
        else:
            waypoint = Vector3()
            waypoint.x = float(self.values['-X-'])
            waypoint.y = float(self.values['-Y-'])
            waypoint.z = float(self.values['-Z-'])
            self.final_dest_x = waypoint.x
            self.final_dest_y = waypoint.y
            self.publisher_1.publish(waypoint)
        invalid = False

    def set_ocean_current(self):
        """Set ocean current"""
        ## Checking if the current coordinates are valid
        invalid = False
        try:
            float(self.values['-CUR_X-'])
            float(self.values['-CUR_Y-'])
            float(self.values['-CUR_Z-'])
            if(float(self.values['-CUR_X-']) > 2 or float(self.values['-CUR_X-']) < -2
                or float(self.values['-CUR_Y-']) > 2 or float(self.values['-CUR_Y-']) < -2
                or float(self.values['-CUR_Z-']) > 2 or float(self.values['-CUR_Z-']) < -2):
                invalid = True
        except ValueError:
            invalid = True
        ## Settnig the limits for the current coordinates

        if(invalid):
            sg.popup('Please enter a valid number for the current\nMax: 2.0\nMin: -2.0!',background_color="yellow", text_color="black", font="Helvetica 14", title="Warning")
        else:
            os.system("gz topic -t /ocean_current -m gz.msgs.Vector3d -p 'x: {}, y:{}, z:{}'".format(self.values['-CUR_Y-'], self.values['-CUR_X-'], (-float(self.values['-CUR_Z-']))))
            
        invalid = False

    def JOY_mode(self,useable_col, unuseable_col):
        """Setting up GUI for joystick control mode"""
        self.window['-STD_TEST_1-'].update(button_color = unuseable_col)
        self.window['-STD_TEST_2-'].update(button_color = unuseable_col)
        self.window['-STD_TEST_3-'].update(button_color = unuseable_col)
        self.window['-STD_TEST_4-'].update(button_color = unuseable_col)
        self.window['-INITIALIZE-'].update(button_color = unuseable_col)
        self.window['-STD_TEST_SEQUENCE-'].update(button_color = unuseable_col)
        self.window['-STD_TEST_1-'].update(disabled=True)
        self.window['-STD_TEST_2-'].update(disabled=True)
        self.window['-STD_TEST_3-'].update(disabled=True)
        self.window['-STD_TEST_4-'].update(disabled=True)
        self.window['-INITIALIZE-'].update(disabled=True)
        self.window['-STD_TEST_SEQUENCE-'].update(disabled=True)

        self.window['-SET_P-'].update(button_color = unuseable_col)
        self.window['-SET_P-'].update(disabled=True)

        self.window['-X-'].update(disabled=True)
        self.window['-Y-'].update(disabled=True)
        self.window['-Z-'].update(disabled=True)

    def TRAJECTORY_mode(self, useable_col, unuseable_col):
        """Setting up GUI for trajectory control mode"""
        self.window['-STD_TEST_1-'].update(button_color = unuseable_col)
        self.window['-STD_TEST_2-'].update(button_color = unuseable_col)
        self.window['-STD_TEST_3-'].update(button_color = unuseable_col)
        self.window['-STD_TEST_4-'].update(button_color = unuseable_col)
        self.window['-INITIALIZE-'].update(button_color = unuseable_col)
        self.window['-STD_TEST_SEQUENCE-'].update(button_color = unuseable_col)
        self.window['-STD_TEST_1-'].update(disabled=True)
        self.window['-STD_TEST_2-'].update(disabled=True)
        self.window['-STD_TEST_3-'].update(disabled=True)
        self.window['-STD_TEST_4-'].update(disabled=True)
        self.window['-INITIALIZE-'].update(disabled=True)
        self.window['-STD_TEST_SEQUENCE-'].update(disabled=True)

        self.window['-SET_P-'].update(button_color = useable_col)
        self.window['-SET_P-'].update(disabled=False)

        self.window['-X-'].update(disabled=False)
        self.window['-Y-'].update(disabled=False)
        self.window['-Z-'].update(disabled=False)

    def STANDARD_TEST_mode(self, useable_col, unuseable_col):
        """Setting up GUI for standard test control mode"""
        self.window['-STD_TEST_1-'].update(button_color = useable_col)
        self.window['-STD_TEST_2-'].update(button_color = useable_col)
        self.window['-STD_TEST_3-'].update(button_color = useable_col)
        self.window['-STD_TEST_4-'].update(button_color = useable_col)
        self.window['-INITIALIZE-'].update(button_color = useable_col)
        self.window['-STD_TEST_SEQUENCE-'].update(button_color = useable_col)
        self.window['-STD_TEST_1-'].update(disabled=False)
        self.window['-STD_TEST_2-'].update(disabled=False)
        self.window['-STD_TEST_3-'].update(disabled=False)
        self.window['-STD_TEST_4-'].update(disabled=False)
        self.window['-INITIALIZE-'].update(disabled=False)
        self.window['-STD_TEST_SEQUENCE-'].update(disabled=False)

        self.window['-SET_P-'].update(button_color = unuseable_col)
        self.window['-SET_P-'].update(disabled=True)

        self.window['-X-'].update(disabled=True)
        self.window['-Y-'].update(disabled=True)
        self.window['-Z-'].update(disabled=True)

    def ROV2_odom_callback(self, msg):
        """Callback function for odometry2 topic"""
        self.pos2 = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
    
    def ROV3_odom_callback(self, msg):
        """Callback function for odometry3 topic"""
        self.pos3 = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
    
    def ref_callback(self, msg):
        """Callback function for reference topic"""
        self.traj_reference = [msg.x, msg.y, msg.z]
    
    def angle_callback2to3(self, msg):
        """Callback function for angle2to3 topic"""
        if(self.FOV_limit > msg.data):
            self.window['-ANGLE_23-'].update(background_color = '#0000B9', text_color="white")
        else:
            self.window['-ANGLE_23-'].update(background_color = 'yellow', text_color="black")
        self.window['-ANGLE_23-'].update("%.2f"%msg.data)
        self.angle2to3 = msg.data
        
    def angle_callback2to4(self, msg):
        """Callback function for angle3to2 topic"""
        if(self.FOV_limit > msg.data):
            self.window['-ANGLE_24-'].update(background_color = '#0000B9', text_color="white")
        else:
            self.window['-ANGLE_24-'].update(background_color = 'yellow', text_color="black")
        self.window['-ANGLE_24-'].update("%.2f"%msg.data)

    def angle_callback3to2(self, msg):
        """Callback function for angle2to3 topic"""
        if(self.FOV_limit > msg.data):
            self.window['-ANGLE_32-'].update(background_color = '#0000B9', text_color="white")
        else:
            self.window['-ANGLE_32-'].update(background_color = 'yellow', text_color="black")
        self.window['-ANGLE_32-'].update("%.2f"%msg.data)
        self.angle3to2 = msg.data

    def angle_callback3to4(self, msg):
        """Callback function for angle3to4 topic"""
        if(self.FOV_limit > msg.data):
            self.window['-ANGLE_34-'].update(background_color = '#0000B9', text_color="white")
        else:
            self.window['-ANGLE_34-'].update(background_color = 'yellow', text_color="black")
        self.window['-ANGLE_34-'].update("%.2f"%msg.data)
    
    def angle_callback4to2(self, msg):
        """Callback function for angle4to2 topic"""
        if(self.FOV_limit > msg.data):
            self.window['-ANGLE_42-'].update(background_color = '#0000B9', text_color="white")
        else:
            self.window['-ANGLE_42-'].update(background_color = 'yellow', text_color="black")
        self.window['-ANGLE_42-'].update("%.2f"%msg.data)

    def angle_callback4to3(self, msg):
        """Callback function for angle4to3 topic"""
        if(self.FOV_limit > msg.data):
            self.window['-ANGLE_43-'].update(background_color = '#0000B9', text_color="white")
        else:
            self.window['-ANGLE_43-'].update(background_color = 'yellow', text_color="black")
        self.window['-ANGLE_43-'].update("%.2f"%msg.data)

    def std_test_sub_callback(self, msg):
        self.std_test_ready_next = msg.data

    def open_window(self):
        """Opens a new window with parameters"""
        cwd = os.getcwd()
        params_path = cwd + "/src/mpc_controller/params/params.yaml"

        with open(str(params_path)) as my_file:
            docu = []
            for line in my_file:
                docu.append(line)
        layout = [
            [sg.Text("Parameters", font=("Helvetica", 25), text_color="white")],
            [sg.Text(docu[2])],
            [sg.Text(docu[3])],
            [sg.Text(docu[4])],
            [sg.Text(docu[5])],
            [sg.Text(docu[6])],
            [sg.Text(docu[7])],
            [sg.Text(docu[8])],
            #[sg.Text(str(pwd))]
            [sg.Button("Exit", key="Exit")]
        ]
        window = sg.Window("Parameters", layout, modal=True)
        choice = None
        while True:
            event, values = window.read()
            if event == "Exit" or event == sg.WIN_CLOSED:
                break
        
        window.close()
        

    def draw_figure(self,canvas, figure):
        figure_canvas_agg = FigureCanvasTkAgg(figure, canvas)
        figure_canvas_agg.draw()
        figure_canvas_agg.get_tk_widget().pack(side='top', fill='both', expand=1)
        return figure_canvas_agg


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = GUI()

    rclpy.spin(minimal_subscriber)

    sg.window.close()
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()