import time,math
import numpy as np
LINE_BOTTOM_FORCE_CONST = 1
LINE_TOP_FORCE_CONST = 1
# a time cnfd. of x means 0% cnfd. at x seconds.unfortunately no way to
#completely disable this
TIME_CONFIDENCE_CONST = 0.25

#Dont think these are being used anymore
TOP_CONFIDENCE_CONST = 50      # a confidence of 50 means 0% confidence
BOTTOM_CONFIDENCE_CONST = 50   # when the line is at the edge. 25 means 50%
MIN_TURN_PWR=20
MIN_FWD_PWR=30
# might need to reset these to 1
HORZ_FORCE_CONST = 2
VERT_FORCE_CONST = 3

TURN_TIME_90 = 3 #seconds


from scipy.interpolate import interp1d

class DirectionControl:
    def __init__(self):
        self.current_sensor_confidence=0
        self.current_movement_confidence=0
        self.line_force0_last = 0
        self.last_elapsed_time = 0
        self.pr = np.array([0,0])
        self.v_last = 0
        self.omega_last = 0
        self.last_barcode = None
        self.do_barcode_routine=False
        self.djikstra_turn = -1
        self.sweep_start_time=None
    
    def step(self,controller_state,camera_state):
        #might offload this into calc_line_force
        
        USE_LINE_FOLLOW = controller_state['triangle']
        if USE_LINE_FOLLOW:
            
            if self.is_new_barcode(camera_state['barcode']):
                print("FOUND NEW BARCODE")
                self.reset_dead_reckon_timer()
                self.dead_reckon_straight = 5
                self.dead_reckon_turn = 3
                self.do_barcode_routine = True
            
            if self.do_barcode_routine:
                vert,horz = self.barcode_routine(camera_state)
                
            elif camera_state['top'] is None or camera_state['confidence']==0:
                vert,horz = self.do_lost_sweep(camera_state)
                
            else: #usual line following
                vert,horz = self.calc_line_force(camera_state)
            
            
        else:
            DS4_vert = controller_state['vert']
            DS4_horz = controller_state['horz']
            vert,horz = DS4_vert,DS4_horz
        
        return vert/100, horz/100
    
    def reset_dead_reckon_timer(self):
        self.dead_reckon_timer = time.time()
    
    def barcode_routine(self,camera_state):
        log_msg = ""
        current_time = time.time()
        
        # dead reckon for x seconds
        if self.dead_reckon_straight is not None:
            log_msg+="DEAD RECKON STRAIGHT"
            if (current_time - self.dead_reckon_timer)<self.dead_reckon_straight:
                vert,horz = 50,0
            else:
                self.dead_reckon_straight = None
                self.reset_dead_reckon_timer()
                vert,horz = 0,0
                
        elif self.dead_reckon_turn is not None:
            log_msg+="DEAD RECKON TURN"
            if (current_time - self.dead_reckon_timer)<self.dead_reckon_turn:
                vert,horz = 0,self.djikstra_turn*100
            else:
                self.dead_reckon_turn = None
                self.dead_reckon_timer = None
                vert,horz = 0,0
                
        else:
            self.do_barcode_routine = False
            vert,horz = 0,0
        return -vert,horz
    
    
    def do_lost_sweep(self,camera_state):
        print("DOING LOST SWEEP")
        return 0,0
        
    def is_new_barcode(self,barcode):
        #idk if this will work I'm a little tired
        ret_bool = (barcode is not None) and (barcode != self.last_barcode)
        self.last_barcode = barcode
        return ret_bool
    
    def calc_line_force(self,camera_state):
        log_msg=""
        
        line_top = camera_state['top']        # as percentage of viewing area
        line_bottom = camera_state['bottom']  # from bottom or center
        last_time = camera_state['message_time']
        line_confidence = camera_state['confidence']/100
        
        line_force = (np.array(line_top) + np.array(line_bottom))/2
        # Line confidence calculation. Confidence restricts the
        # vertical movement, currently leaves hz movement as it is
        current_time = time.time()
        elapsed_time = current_time-last_time
        # check if this frame is new            
        
        time_confidence = max(1 - elapsed_time/TIME_CONFIDENCE_CONST,0)
        time_confidence = min(max(time_confidence,0),1)        
        sensor_confidence = line_confidence * time_confidence
        stp = 0.001
        if sensor_confidence>self.current_sensor_confidence+stp:
            self.current_sensor_confidence+=stp
        else:
            self.current_sensor_confidence = sensor_confidence
        
        
        theta = np.arctan((line_top[1]-line_bottom[1])/(line_top[0]-line_bottom[0]))
        theta_confidence = min(max(1-abs(theta / (np.pi / 2)),0),1)
        
        lateral_confidence = max(50-abs(line_force[1]),0)/50
        #         movement_confidence = (100-abs(line_top[1])/TOP_CONFIDENCE_CONST)*\
#                               (100-abs(line_bottom[1])/BOTTOM_CONFIDENCE_CONST)
        movement_confidence = theta_confidence*lateral_confidence#min(max(movement_confidence,0),1)

        stp = 0.01
        if movement_confidence>self.current_movement_confidence+stp:
            self.current_movement_confidence+=stp
        else:
            self.current_movement_confidence = movement_confidence
            
    
            
#         print(self.current_confidence)
        # compute the forces.
#         Kp = 0.9
#         Kd = 0
#         dline_force0 = line_force[0] - self.line_force0_last
#         self.line_force0_last = line_force[0]
#         line_force[0] = Kp * line_force[0] + Kd * dline_force0
        log_msg+=f"| line conf:{int(line_confidence*100) : 3}"
        log_msg+=f"| time conf:{int(time_confidence*100) : 3}"
        log_msg+=f"|theta conf:{int(theta_confidence*100) : 3}"
        log_msg+=f"|line force orig:{tuple(line_force.astype(int))}:5"
        
#         line_force[0]*= max(min(((1 - theta_conf) * (1 + theta_conf))**6,1),0.1)
#         print("v after theta:",line_force[0])
        line_force[0]*= VERT_FORCE_CONST * \
                        self.current_sensor_confidence * \
                        self.current_movement_confidence
#         print("v after horz adj:",line_force[0])
        line_force[1]*= HORZ_FORCE_CONST * \
                        self.current_sensor_confidence
        
        log_msg+=f"|adj force:{tuple(line_force.astype(int))}"
        
        if abs(line_force[1])>1 and line_force[0]<5:
            line_force[1]=math.copysign(MIN_TURN_PWR,line_force[1])
            
        elif line_force[0]>5:
            line_force[0]=max(line_force[0],MIN_FWD_PWR)
    
        log_msg+=f"|fin force:{tuple(line_force.astype(int))}"

        
#         if theta_conf<0.1:
#             line_force[0] = max(line_force[1],0.1)*VERT_FORCE_CONST
        
#         print("CONF",self.current_confidence)
        if len(log_msg)!=0:
            print(log_msg)
        return -line_force[0], line_force[1]
        

    
    def get_path_plan(camera_state,current_time):
        line_top = camera_state['top']        # as percentage of viewing area
        line_bottom = camera_state['bottom']  # from bottom or center
        last_time = camera_state['message_time']
        line_confidence = camera_state['confidence']/100
        
        elapsed_time = current_time-last_time
        #check for a new time
        if self.last_elapsed_time > elapsed_time:
            print("GENERATING NEW TRAJECTORY")
            self.fit_thetas = generate_path_plan(line_top,line_bottom,elapsed_time)
            dt = elapsed_time
            self.cum_dist = 0
        else:
            print("INTERPOLATING FROM OLD TRAJECTORY")
            dt = self.last_elapsed_time-elapsed_time
        horz=self.interpolate_from_path_plan(dt)
        
        self.last_elapsed_time = elapsed_time
        return horz
    
    def update_path_plan(self,dt):
        self.theta = self.theta + omega_last * dt
        self.pr = self.pr + self.v_last * dt
        
        desired_theta = self.fit_thetas(self.pr[0])
        
        desired_omega = (desired_theta-self.theta)/dt
        
        return desired_omega
        
        
    def generate_path_plan(self,line_top,line_bottom,elapsed_time):
        p1,p2 = line_bottom,line_top
        vlcty = self.v_last
        omega = self.omega_last
        
        pr = self.pr = np.array(0,0)  #position of robot
        pr2 = pr+vlcty*elapsed_time
        
        # fit a trajectory
        xs = (pr[0],pr2[0],0.9*(p2[0]-p1[0])+p1[0],p2[0],)
        ys = (pr[1],pr2[1],0.9*(p2[1]-p1[1])+p1[1],p2[1],)
        coeff = np.polyfit(ys,xs,3)
        
        fit = lambda y: sum([coeff[i]*y**(len(coeff)-i-1) for i in range(len(coeff))])
        
        y = np.arange(p1[0],p2[0])
        x = [fit(y) for y in y]
        
        # pull dist and thetas from this
        dist = np.array([((x[i+1]-x[i])**2+(y[i+1]-y[i])**2)**0.5 for i in range(len(x)-1)])
        cum_dist = np.array([sum(dist[:i]) for i in range(len(dist))])
        
        thetas = [np.arctan2(x[i+1]-x[i],y[i+1]-y[i]) for i in range(len(x)-1)]

        fit_thetas = interp1d(y,thetas)
        
        self.pr = pr2
        self.theta = 0 + omega*elapsed_time
        return fit_thetas
    
    def calc_line_force_pathplan(self,camera_state):
        line_top = camera_state['top']        # as percentage of viewing area
        line_bottom = camera_state['bottom']  # from bottom or center
        last_time = camera_state['message_time']
        line_confidence = camera_state['confidence']/100
        
        current_time = time.time()
        elapsed_time = current_time-last_time
        
        time_confidence = max(1 - elapsed_time/TIME_CONFIDENCE_CONST,0)
        time_confidence = min(max(time_confidence,0),1)
        
        confidence = line_confidence * time_confidence
        
        stp = 0.01
        if confidence>self.current_confidence+stp:
            self.current_confidence+=stp
        else:
            self.current_confidence = confidence
            
        horz = get_path_plan(camera_state,current_time)
        
        vert = 1-line_top[0]
                
        