import rclpy
from rclpy.node import Node
import PySimpleGUI as sg
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import os

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32

class GUI(Node):

    def __init__(self):
        ###### INIT ROS2 ############################################################################
        super().__init__('GUI')
        self.declare_parameter('fleet_quantity')
        self.n_agents = self.get_parameter('fleet_quantity').get_parameter_value().integer_value
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
        if(self.n_agents > 1):
            self.subscription = self.create_subscription(
                Odometry,
                '/bluerov2_pid/bluerov3/observer/nlo/odom_ned',
                self.ROV2_odom_callback,
                10)
        if(self.n_agents > 2):
            self.subscription = self.create_subscription(
                Odometry,
                '/bluerov2_pid/bluerov4/observer/nlo/odom_ned',
                self.ROV3_odom_callback,
                10)
        self.subscription  # prevent unused variable warning
        self.publisher_1 = self.create_publisher(Vector3, '/trajectory_waypoints', 10)
        self.publisher_control_mode = self.create_publisher(Int32, '/control_mode', 10)

        ###### INIT PLOT ############################################################################
        w, h = figsize = (10, 10)     # figure size
        self.fig, self.ax = plt.subplots(figsize=figsize)
        dpi = self.fig.get_dpi()
        self.size = (int(w*dpi), int(h*dpi))
        self.ax.grid(True)
        self.ax.set(xlim=(-10, 10), ylim=(-10, 10))
        self.ax.set_title('Birds eye view of sea', fontdict={'fontsize': 20})
        self.ax.set_xlabel('x', fontdict={'fontsize': 15})
        self.ax.set_ylabel('y', fontdict={'fontsize': 15})

        ###### INIT PYSIMPLEGUI ######################################################################
        self.setup_layout()
        self.window = sg.Window('ROV Simulator GUI', self.layout, finalize=True,size=(1800, 900), element_justification='center')
        self.canvas_elem = self.window['-CANVAS-']
        self.canvas = self.canvas_elem.TKCanvas
        # draw the intitial scatter plot
        self.fig_agg = self.draw_figure(self.canvas, self.fig)

        ## initialize the variables to avoid errors
        self.traj_reference = [0,0,0]
        self.pos2 = [0,0,0]
        self.pos3 = [0,0,0]

        self.trajectory_log_1 = [[],[]]
        self.trajectory_log_2 = [[],[]]
        self.trajectory_log_3 = [[],[]]

        self.final_dest_x = 0
        self.final_dest_y = 0



    def ROV1_main_callback(self, msg):
        """Callback from main odometry. Also main PySimpleGUI loop"""
        ### get the position of the ROV
        self.odom1 = msg.pose.pose.position
        event, self.values = self.window.read(timeout=5)
        if event in ('-EXIT-', None): # if user closes window or clicks cancel. Stop the program
            exit(69)
        if event == '-SET_P-': # if user sets a new waypoint
            self.publish_waypoint()
        if event == '-READ-':
            self.open_window()
        mode = Int32()
        if self.values['-AUTO-'] == True:
            mode.data = 1
        else:
            mode.data = 0
        self.publisher_control_mode.publish(mode)

        self.reinitialize_plot()

        ## plot the reference and final destination
        self.ax.scatter(self.traj_reference[0], self.traj_reference[1], c='black', s=20, label='Reference', marker='x')
        self.ax.scatter(self.final_dest_x, self.final_dest_y, c='black', s=20, label='Final dest.')

        ## update the canvas with ROV1
        if self.values['-TRAJ1-'] == True:
            if(len(self.trajectory_log_1[0]) > 2000):
                self.trajectory_log_1[0].pop(0)
                self.trajectory_log_1[1].pop(0)
            self.trajectory_log_1[0].append(self.odom1.x)
            self.trajectory_log_1[1].append(self.odom1.y)
        else:
            self.trajectory_log_1 = [[],[]]
        self.ax.scatter(self.odom1.x, self.odom1.y, c='blue', s=20, label='ROV 1')
        self.ax.plot(self.trajectory_log_1[0], self.trajectory_log_1[1], c='blue')
        ## update the canvas with ROV2
        if self.n_agents > 1:
            if self.values['-TRAJ2-'] == True:
                if(len(self.trajectory_log_2[0]) > 2000):
                    self.trajectory_log_1[0].pop(0)
                    self.trajectory_log_1[1].pop(0)
                self.trajectory_log_2[0].append(self.pos2[0])
                self.trajectory_log_2[1].append(self.pos2[1])
            else:
                self.trajectory_log_2 = [[],[]]
            self.ax.scatter(self.pos2[0], self.pos2[1], c='green', s=20, label='ROV 2')
            self.ax.plot(self.trajectory_log_2[0], self.trajectory_log_2[1], c='green')
        ## update the canvas with ROV3
        if self.n_agents > 2:
            if self.values['-TRAJ3-'] == True:
                if(len(self.trajectory_log_3[0]) > 2000):
                    self.trajectory_log_1[0].pop(0)
                    self.trajectory_log_1[1].pop(0)
                self.trajectory_log_3[0].append(self.pos3[0])
                self.trajectory_log_3[1].append(self.pos3[1])
            else:
                self.trajectory_log_3 = [[],[]]
            self.ax.scatter(self.pos3[0], self.pos3[1], c='red', s=20, label='ROV 3')
            self.ax.plot(self.trajectory_log_3[0], self.trajectory_log_3[1], c='red')
        ## Update the canvas with the new plot
        self.update_xyz_GUI_indication()
        self.ax.legend()
        self.fig_agg.draw()

    def update_xyz_GUI_indication(self):
        """Update the xyz values in the GUI"""
        self.window['-X_visual-'].update("%.2f"%self.odom1.x)
        self.window['-Y_visual-'].update("%.2f"%self.odom1.y)
        self.window['-Z_visual-'].update("%.2f"%self.odom1.z)
        if(self.n_agents > 1):
            self.window['-X_visual2-'].update("%.2f"%self.pos2[0])
            self.window['-Y_visual2-'].update("%.2f"%self.pos2[1])
            self.window['-Z_visual2-'].update("%.2f"%self.pos2[2])
        if(self.n_agents > 2):
            self.window['-X_visual3-'].update("%.2f"%self.pos3[0])
            self.window['-Y_visual3-'].update("%.2f"%self.pos3[1])
            self.window['-Z_visual3-'].update("%.2f"%self.pos3[2])

    def reinitialize_plot(self):
        """Setup the plot. Runs every iteration of the main loop"""
        self.ax.cla()
        self.ax.grid(True)
        self.ax.set(xlim=(-20, 20), ylim=(-20, 20))
        self.ax.set_title('Birds eye view of sea', fontdict={'fontsize': 20})
        self.ax.set_xlabel('x', fontdict={'fontsize': 15})
        self.ax.set_ylabel('y', fontdict={'fontsize': 15})

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
            [sg.Text('Position waypoint', size=(50, 1), justification='center', font=(font, 12, "bold"),text_color=text_col, background_color=unclickable_col)],
            [sg.Text('X:',font=font, size=(4, 1),text_color=text_col, background_color=unclickable_col), 
            sg.InputText(size=(34, 1), pad=((10, 0), 3), font=font, key='-X-',text_color=clickable_text_col, background_color=clickable_backgr_col, default_text='0'),
            sg.Text('-20 < X < 20',font=font, size=(10, 1),text_color=text_col, background_color=background_col),
            ],
            [sg.Text('Y:',font=font, size=(4, 1),text_color=text_col, background_color=unclickable_col), 
            sg.InputText(size=(34, 1), pad=((10, 0), 3), font=font, key='-Y-',text_color=clickable_text_col, background_color=clickable_backgr_col, default_text='0'),
            sg.Text('-20 < Y < 20',font=font, size=(10, 1),text_color=text_col, background_color=background_col),
            ],
            [sg.Text('Z:',font=font, size=(4, 1),text_color=text_col, background_color=unclickable_col), 
            sg.InputText(size=(34, 1), pad=((10, 0), 3), font=font, key='-Z-',text_color=clickable_text_col, background_color=clickable_backgr_col, default_text='0'),
            sg.Text('   0 < Z < 15',font=font, size=(10, 1),text_color=text_col, background_color=background_col),],
            [sg.Button('Set position', size=(38, 2), font=font, key='-SET_P-', button_color=('black',button_col))],
            [sg.Text('', background_color=background_col)],
            [sg.Text('Attitude reference', size=(50, 1), justification='center', font=(font, 12, "bold"),text_color=text_col, background_color=unclickable_col)],
            [sg.Text('η ',font=font, size=(4, 1),text_color=text_col, background_color=unclickable_col), 
            sg.InputText(size=(34, 1), pad=((10, 0), 3), font=font, key='-ETA-',text_color=clickable_text_col, background_color=clickable_backgr_col, default_text='1'),
            sg.Text('            ',font=font, size=(10, 1),text_color=text_col, background_color=background_col),
            ],
            [sg.Text('ε1',font=font, size=(4, 1),text_color=text_col, background_color=unclickable_col), 
            sg.InputText(size=(34, 1), pad=((10, 0), 3), font=font, key='-EPS1-',text_color=clickable_text_col, background_color=clickable_backgr_col, default_text='0'),
            sg.Text('            ',font=font, size=(10, 1),text_color=text_col, background_color=background_col),
            ],
            [sg.Text('ε2',font=font, size=(4, 1),text_color=text_col, background_color=unclickable_col), 
            sg.InputText(size=(34, 1), pad=((10, 0), 3), font=font, key='-EPS2-',text_color=clickable_text_col, background_color=clickable_backgr_col, default_text='0'),
            sg.Text('            ',font=font, size=(10, 1),text_color=text_col, background_color=background_col),
            ],
            [sg.Text('ε3',font=font, size=(4, 1),text_color=text_col, background_color=unclickable_col), 
            sg.InputText(size=(34, 1), pad=((10, 0), 3), font=font, key='-EPS3-',text_color=clickable_text_col, background_color=clickable_backgr_col, default_text='0'),
            sg.Text('            ',font=font, size=(10, 1),text_color=text_col, background_color=background_col),
            ],
            [sg.Button('Set attitude', size=(38, 2), font=font, key='-SET_A-', button_color=('black', button_col))],
            [sg.Text('', background_color=background_col)],
            [sg.Text('O', background_color=background_col, size=(5, 1), text_color=background_col, justification='center', font=font),
            sg.Text('ROV1', size=(ROV_col_width, 1), justification='center', font=font,text_color=text_col, background_color=unclickable_col),
            sg.Text('ROV2', size=(ROV_col_width, 1), justification='center', font=font,text_color=text_col, background_color=unclickable_col) if self.n_agents > 1 else sg.Text('',size=(0,0), background_color=background_col),
            sg.Text('ROV3', size=(ROV_col_width, 1), justification='center', font=font,text_color=text_col, background_color=unclickable_col) if self.n_agents > 2 else sg.Text('',size=(0,0), background_color=background_col),
            ],
            [sg.Text('X', size=(5, 1), justification='center', font=font,text_color=text_col, background_color=unclickable_col),
            sg.Text('X', size=(ROV_col_width, 1), justification='center', font=font, key='-X_visual-',text_color=text_col, background_color=ind_text_col),
            sg.Text('X', size=(ROV_col_width, 1), justification='center', font=font, key='-X_visual2-',text_color=text_col, background_color=ind_text_col) if self.n_agents > 1 else sg.Text('',size=(0,0), background_color=background_col),
            sg.Text('X', size=(ROV_col_width, 1), justification='center', font=font, key='-X_visual3-',text_color=text_col, background_color=ind_text_col) if self.n_agents > 2 else sg.Text('',size=(0,0), background_color=background_col),
            ],
            [sg.Text('Y', size=(5, 1), justification='center', font=font,text_color=text_col, background_color=unclickable_col),
            sg.Text('Y', size=(ROV_col_width, 1), justification='center', font=font, key='-Y_visual-',text_color=text_col, background_color=ind_text_col),
            sg.Text('Y', size=(ROV_col_width, 1), justification='center', font=font, key='-Y_visual2-',text_color=text_col, background_color=ind_text_col) if self.n_agents > 1 else sg.Text('',size=(0,0), background_color=background_col),
            sg.Text('Y', size=(ROV_col_width, 1), justification='center', font=font, key='-Y_visual3-',text_color=text_col, background_color=ind_text_col) if self.n_agents > 2 else sg.Text('',size=(0,0), background_color=background_col),
            ],
            [sg.Text('Z', size=(5, 1), justification='center', font=font,text_color=text_col, background_color=unclickable_col),
            sg.Text('Z', size=(ROV_col_width, 1), justification='center', font=font, key='-Z_visual-',text_color=text_col, background_color=ind_text_col),
            sg.Text('Z', size=(ROV_col_width, 1), justification='center', font=font, key='-Z_visual2-',text_color=text_col, background_color=ind_text_col) if self.n_agents > 1 else sg.Text('',size=(0,0), background_color=background_col),
            sg.Text('Z', size=(ROV_col_width, 1), justification='center', font=font, key='-Z_visual3-',text_color=text_col, background_color=ind_text_col) if self.n_agents > 2 else sg.Text('',size=(0,0), background_color=background_col),
            ],
            [sg.Text('Plot\nPath', size=(5, 2), justification='center', font=font,text_color=text_col, background_color=unclickable_col),
            sg.Text('', size=(checkbox_spacing_w, 2), background_color=background_col, font=font,text_color=text_col),
            sg.Checkbox('', default=False, key='-TRAJ1-',text_color="black", font=font, background_color=button_col),
            sg.Text('', size=(checkbox_spacing_w-1, 2), background_color=background_col, font=font,text_color=text_col) if self.n_agents > 1 else sg.Text('',size=(0,0), background_color=background_col),
            sg.Text('', size=(checkbox_spacing_w+2, 2), background_color=background_col, font=font,text_color=text_col) if self.n_agents > 1 else sg.Text('',size=(0,0), background_color=background_col),
            sg.Checkbox('', default=False, key='-TRAJ2-',text_color="black", font=font, background_color=button_col) if self.n_agents > 1 else sg.Text('',size=(0,0), background_color=background_col),
            sg.Text('', size=(checkbox_spacing_w-2, 2), background_color=background_col, font=font,text_color=text_col) if self.n_agents > 1 else sg.Text('',size=(0,0), background_color=background_col),
            sg.Text('', size=(checkbox_spacing_w+3, 2), background_color=background_col, font=font,text_color=text_col) if self.n_agents > 2 else sg.Text('',size=(0,0), background_color=background_col),
            sg.Checkbox('', default=False, key='-TRAJ3-',text_color="black", font=font, background_color=button_col) if self.n_agents > 2 else sg.Text('',size=(0,0), background_color=background_col),  
            sg.Text('', size=(checkbox_spacing_w+1, 2), background_color=background_col, font=font,text_color=text_col) if self.n_agents > 2 else sg.Text('',size=(0,0), background_color=background_col),    
            ],
            [sg.Button('Read Parameters', size=(20, 2), font=font, key='-READ-', button_color=('white', 'green'), pad=((0, 0), (10, 0)))],
            [sg.Radio('Manual (Joystick)               ', "Control_mode", default=False,text_color="black", font=font, background_color=button_col, size=(30, 1))],
            [sg.Radio('Autonomous (Trajectory planning)', "Control_mode", key='-AUTO-',text_color="black", font=font, background_color=button_col, default=True, size=(30, 1))],
            [sg.Text('', background_color=background_col)],
            [sg.Button('Exit', size=(20, 2), font=font, key='-EXIT-', button_color=('white', 'red'), pad=((0, 0), (10, 0)))]
            ]
        self.sec_col = [
            [sg.Canvas(size=self.size, key='-CANVAS-', background_color='white')],
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
            sg.popup('Please enter a valid number for the waypoint coordinates!',background_color="yellow", text_color="black", font="Helvetica 14", title="Warning")
        else:
            waypoint = Vector3()
            waypoint.x = float(self.values['-X-'])
            waypoint.y = float(self.values['-Y-'])
            waypoint.z = float(self.values['-Z-'])
            self.final_dest_x = waypoint.x
            self.final_dest_y = waypoint.y
            self.publisher_1.publish(waypoint)
        invalid = False

    
    def ROV2_odom_callback(self, msg):
        """Callback function for odometry2 topic"""
        self.pos2 = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
    
    def ROV3_odom_callback(self, msg):
        """Callback function for odometry3 topic"""
        self.pos3 = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
    
    def ref_callback(self, msg):
        """Callback function for reference topic"""
        self.traj_reference = [msg.x, msg.y, msg.z]

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

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sg.window.close()
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()