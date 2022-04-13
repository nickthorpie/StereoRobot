"""
DS4_cartcontrol.py
DESCRIPTION:
    Controls a sabertooth dual motor with a sony DS4 remote on a RPi4.
    
NOTES:
    Multithreading is used to listen to the controller in a separate process.
    A Queue is used to deliver the most recent command as a JSON string.
    PROGRAM 1: Controller Client
        Reads the controller using pyPS4Controller Library.
        Changes in left joystick position are written to queue as 'vert' and 'horz'.
        Joystick map is +/-32767
        Be sure to connect DS4 in bluetooth first using RPI's 
        
    PROGRAM 2: Motor Server
        Reads out of the queue, and sends PWM signals to the two motor driver signals
        Currently all pinouts are hardcoded as GPIO 12 and 13. Duty cycle 50%.
        The sabertooth switch configuation is in RC mode as follows:
        | 1 | 2 | 3 | 4 | 5 | 6 |
        |DWN| UP| UP| UP| UP|DWN|
        In RC Mode, motor direction is a function of PWM frequency. These may/will
        change depending on configuration (input voltage, driver, duty cycle, gremlins).
        The mapping of frequencies to output voltage were collected by hand in
        SABERTOOTH_VOLTAGE_FREQ_MAP.ipynb, and are fit with a quadratic function.
        
"""

# FIFO = 'mypipe'
# 
# try:
#     os.mkfifo(FIFO)
# except OSError as oe: 
#     if oe.errno != errno.EEXIST:
#         raise

import time
from multiprocessing import Queue, Process


# # PROGRAM 1: Controller client
# from pyPS4Controller.controller import Controller
# 
# class MyController(Controller):
#     def __init__(self,pipe, **kwargs):
#         Controller.__init__(self, **kwargs)
#         self.control = {'vert':0,'horz':0}
#         self.pipe = pipe
#         self.write_controller()
#         
#     def write_controller(self):
#         self.pipe.put(self.control)
#         
#         
#     def on_x_press(self):
#        print("Hello world")
# 
#     def on_x_release(self):
#        print("Goodbye world")
#     
#     def on_L3_left(self,value):
#         self.control['horz'] = value
#         self.write_controller()
#         
#     def on_L3_right(self,value):
#         self.control['horz'] = value
#         self.write_controller()
#         
#     def on_L3_up(self,value):
#         self.control['vert'] = value
#         self.write_controller()
# 
#     def on_L3_down(self,value):
#         self.control['vert'] = value
#         self.write_controller()
#         
#     def on_L3_x_at_rest(self):
#         self.control['horz'] = 0
#         self.write_controller()
#     
#     def on_L3_y_at_rest(self):
#         self.control['vert'] = 0
#         self.write_controller()
# 
# def client(q):
#     controller = MyController(q,interface="/dev/input/js0", connecting_using_ds4drv=False)
#     controller.listen(timeout=60)


## PROGRAM 2: Motor Control

import math
import numpy as np

def accelMotor(current,target,step):
    if current==target:
        return int(target)
    direction = math.copysign(1,target-current)
    next_ = step*direction + current
    if direction > 0:
        if target-next_<0:     #if we overshot:
            return int(target)
        else:
            return int(next_)
    else:
        if target-next_>0:
            return int(target)
        else:
            return int(next_)

# collected data to map frequencies to motor voltages. Tested on Kastar 12v
FREQS = np.array([250,  300, 350, 400, 450, 500,  550])
VOLTS = np.array([    -12.13, -2.6, 0.0,   3, 6.8, 10.9, 11.8])

COEF = np.polyfit(VOLTS,FREQS,2)
freqFun = lambda x: sum(COEF*[x**2,x,1])


PIN_MTR_L = 13
PIN_MTR_R = 12

MTR_PWMduty = int(0.5e6)      # Motor duty cycle range from 0 to 1e6
MTR_ACCEL = 1

JOYSTICK_MAX = 32767  # These are used to map the position of DS4 to a motor voltage
MTR_VOLT_MAX = 10     # V

TURN_TIME_90 = 10 

def motor_differential(vert,horz):
    """
    Takes a vert and horz signal (both from [-1,1]) and splits it up into a left and right
    wheel command. I.e. vert = 1, horz = 0 --> left = 1, right = 1
                        vert = 1, horz = 1 --> left = 0.75, right = 0.25
    """
    L = -(vert -0.5*horz)
    R = -(vert +0.5*horz)
    L = min(max(L,-1),1)
    R = min(max(R,-1),1)
    return L,R

    
# def server(q):
#     # Setup PIGPIO
#     pi = pigpio.pi()
#     pi.set_mode(PIN_MTR_L,pigpio.OUTPUT)
#     pi.set_mode(PIN_MTR_R,pigpio.OUTPUT)
#     
#     L_freq,L_freq_target = 0,0
#     R_freq,R_freq_target = 0,0
#     
#     while True:
#         # read the controller
#         try:
#             message = q.get_nowait()
# #             print('recieved: ', message)
#             controller_state = message
#             
#             DS4_vert = controller_state['vert']/JOYSTICK_MAX
#             DS4_horz = controller_state['horz']/JOYSTICK_MAX
#             
#             L,R = motor_differential(DS4_vert,DS4_horz)
#             
#             L_freq_target = freqFun(L*MTR_VOLT_MAX)
#             R_freq_target = freqFun(R*MTR_VOLT_MAX)
#             
#             
#             
#         except Empty:
#             pass
#         
#         L_freq = accelMotor(L_freq,L_freq_target,MTR_ACCEL)
#         R_freq = accelMotor(R_freq,R_freq_target,MTR_ACCEL)
# #         print(f'left:  L: {L*MTR_VOLT_MAX}  freq:{L_freq}')
# #         print(f'right: R: {R*MTR_VOLT_MAX}  freq:{R_freq}')
#         
#         pi.hardware_PWM(PIN_MTR_L,L_freq,MTR_PWMduty)
#         pi.hardware_PWM(PIN_MTR_R,R_freq,MTR_PWMduty)
        
from stereo_robot.motor_control import DS4Client, SabertoothServer
from stereo_robot.camera_utils import CameraClient
import cv2, pigpio
if __name__=='__main__':
    queue_camera = Queue()
    queue_controller = Queue()
    cam = cv2.VideoCapture(0)
    pi = pigpio.pi()
    process_a = Process(
        target = DS4Client,
        args=(queue_controller,)
    )
    process_b = Process(
        target = SabertoothServer,
        args = (queue_controller,queue_camera,pi)
    )
    process_c = Process(
        target = CameraClient,
        args = (queue_camera,cam)
    )
    process_c.start() #cam takes a second to load up
    time.sleep(1)
    process_a.start()
    process_b.start()    
    print('ctrl+c to exit')
    
    try:
        while True:
            if not (process_a.is_alive() and \
                    process_b.is_alive() and \
                    process_c.is_alive()):
                raise KeyboardInterrupt
            time.sleep(1)
    except KeyboardInterrupt:
        print("Performing Clean Exit")
        pass
    
    process_a.terminate()
    process_b.terminate()
    process_c.terminate()
    
    process_a.join()
    process_b.join()
    process_c.join()

    pi.stop()
# process.start()
# process.join()
# server(queue)
# you can start listening before controller is paired, as long as you pair it within the timeout window

