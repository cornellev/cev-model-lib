from models import KinematicBicycleModel
from visualizer import PygameEngine, BicycleModelVisualizer
import numpy as np

model = KinematicBicycleModel()

# print(model.state)

# for i in range(10):
#     u = np.array([ [1], [np.deg2rad(20)] ])
#     model.update(u, 0.1)


print(model.state)

eng = PygameEngine(800, 640)
eng.add_visualizers([ BicycleModelVisualizer(model) ])

i = 0
j = 0
def fn():
    global i, j
    model.state[2,0] = np.deg2rad(i)
    model.state[3,0] = np.deg2rad(j)
    j+=0.2
    i+=0.1

eng.run(fn)