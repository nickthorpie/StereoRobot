# PROGRAM 1: Controller client
from pyPS4Controller.controller import Controller


JOYSTICK_MAX = 32767  # These are used to map the position of DS4 to a motor voltage

class MyController(Controller):
    def __init__(self,pipe, **kwargs):
        Controller.__init__(self, **kwargs)
        self.control = {'vert':0,'horz':0,'triangle':False,'circle':False}
        self.pipe = pipe
        self.write_controller()
        
    def write_controller(self):
        self.pipe.put(self.control)
        
        
    def on_x_press(self):
       print("Hello world")

    def on_x_release(self):
       print("Goodbye world")
    
    def on_L3_left(self,value):
        self.control['horz'] = 100 * (value/JOYSTICK_MAX)
        self.write_controller()
        
    def on_L3_right(self,value):
        self.control['horz'] = 100 * (value/JOYSTICK_MAX)
        self.write_controller()
        
    def on_L3_up(self,value):
        self.control['vert'] = 100 * (value/JOYSTICK_MAX)
        self.write_controller()

    def on_L3_down(self,value):
        self.control['vert'] = 100 * (value/JOYSTICK_MAX)
        self.write_controller()
        
    def on_L3_x_at_rest(self):
        self.control['horz'] = 0
        self.write_controller()
    
    def on_L3_y_at_rest(self):
        self.control['vert'] = 0
        self.write_controller()
    
    def on_triangle_press(self):
        self.control['triangle'] = True
        self.write_controller()
    
    def on_triangle_release(self):
        self.control['triangle'] = False
        self.write_controller()
        
    def on_circle_press(self):
        self.control['circle'] = True
        self.write_controller()
    
    def on_circle_release(self):
        self.control['circle'] = False
        self.write_controller()
    
    def on_disconnect(self):
        print("User defined disconnect routine")
        self.control['horz'] = 0
        self.control['vert'] = 0
        self.control['circle'] = False
        self.write_controller()
        raise Exception("Lost controller")

def client(q):
    controller = MyController(q,interface="/dev/input/js0", connecting_using_ds4drv=False)
    controller.listen(timeout=60,on_disconnect=controller.on_disconnect)

from multiprocessing import Queue
import time
if __name__ == "__main__":
    q = Queue()
    controller = MyController(q,interface="/dev/input/js0", connecting_using_ds4drv=False)
    controller.listen(timeout=60,on_disconnect=controller.on_disconnect)
    while True:
        print("while Loop")
        time.sleep(1)
