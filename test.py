from models import *
from visualizer import PygameEngine, BicycleModelVisualizer
import numpy as np
import pygame
import time

class KeyboardInputDevice:
    def __init__(self, scale_up, scale_down):
        self.u = np.zeros((2, 1))

        self.scale_up = scale_up
        self.scale_down = scale_down

    def get_input(self):
        return self.u

    def update_with_keys(self, keys):
        # TODO: parameterize maximum controllable velocity
        self.u[0,0] = self.scale_up * (keys[pygame.K_UP] - keys[pygame.K_DOWN])
        self.u[1,0] = self.scale_down * (keys[pygame.K_LEFT] - keys[pygame.K_RIGHT])


def get_environment(which):
    eng = PygameEngine(800, 640)
    last_time = time.time()

    if which == "kinematic_bicycle":
        model = KinematicBicycleModel(0.15)
        kb = KeyboardInputDevice(scale_up=0.3, scale_down=np.deg2rad(30))
        eng.add_visualizers([ BicycleModelVisualizer(model) ])
        eng.add_input_devices([ kb ])

        def fn():
            nonlocal last_time
            curr_time = time.time()
            model.update(kb.get_input(), curr_time - last_time)
            last_time = curr_time

        return lambda: eng.run(fn)
    
    elif which == "dynamic_bicycle":
        model = DynamicBicycleModel(0.15, 2.27, static_friction=0.9, kinetic_friction=0.7)
        kb = KeyboardInputDevice(scale_up=20.8, scale_down=np.deg2rad(30))
        eng.add_visualizers([ BicycleModelVisualizer(model) ])
        eng.add_input_devices([ kb ])

        def fn():
            print(model.state[2,0])
            nonlocal last_time
            curr_time = time.time()
            model.update(kb.get_input(), curr_time - last_time)
            last_time = curr_time


        return lambda: eng.run(fn)
    
    elif which == "voltage_dynamic_bicycle":
        model = VoltageBicycleModel(
            wheelbase=0.28, wheel_radius=0.051, 
            mass=3.4, 
            static_friction=0.7, motor=VoltageBicycleModelMotor())
        kb = KeyboardInputDevice(scale_up=6.0, scale_down=np.deg2rad(30))
        eng.add_visualizers([ BicycleModelVisualizer(model) ])
        eng.add_input_devices([ kb ])

        def fn():
            print(model.state[4,0])
            nonlocal last_time
            curr_time = time.time()
            model.update(kb.get_input(), curr_time - last_time)
            last_time = curr_time


        return lambda: eng.run(fn)

get_environment("voltage_dynamic_bicycle")()

# model = VoltageBicycleModel(
#     wheelbase=0.28, wheel_radius=0.051, 
#     mass=3.4, 
#     static_friction=0.7, motor=VoltageBicycleModelMotor())


# for i in range(10000):
#     if i < 5000:
#         model.update(np.array([ [0.0], [+np.deg2rad(30)] ]), 0.01)
#     else:
#          model.update(np.array([ [0.0], [-np.deg2rad(30)] ]), 0.01)
#     if i%1000 == 0:
#         print(i * 0.01, model.state[4,0])