from models import KinematicBicycleModel
from visualizer import PygameEngine, BicycleModelVisualizer
import numpy as np
import pygame


model = KinematicBicycleModel()

# print(model.state)

# for i in range(10):
#     u = np.array([ [1], [np.deg2rad(20)] ])
#     model.update(u, 0.1)


print(model.state)

class KeyboardInputDevice:
    def __init__(self):
        self.u = np.zeros((2, 1))

    def get_input(self):
        return self.u

    def update_with_keys(self, keys):
        # TODO: parameterize maximum controllable velocity
        self.u[0,0] = 0.3 * (keys[pygame.K_UP] - keys[pygame.K_DOWN])
        self.u[1,0] = np.deg2rad(30) * (keys[pygame.K_LEFT] - keys[pygame.K_RIGHT])


kb = KeyboardInputDevice()

eng = PygameEngine(800, 640)
eng.add_visualizers([ BicycleModelVisualizer(model) ])
eng.add_input_devices([ kb ])

def fn():
    model.update(kb.get_input(), 0.1)

eng.run(fn)