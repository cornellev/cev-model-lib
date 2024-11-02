import numpy as np
from visualizer import BicycleModelVisualizationInfo

class KinematicBicycleModel:
    """ simplest kinematic bicycle model """

    # NUM_STATE_DIMS = 4
    # NUM_INPUT_DIMS = 2

    def __init__(self):
        self.state = np.zeros((4, 1))
        self.wheelbase = 0.15 # meters, 6 inches. Front <-> Rear axis distance

    def update(self, inputs, dt):
        if inputs.shape != (2, 1): raise Exception("incorrect size of inputs")

        dx = np.array([[
            inputs[0,0] * np.cos(self.state[2,0]),
            inputs[0,0] * np.sin(self.state[2,0]),
            (inputs[0,0] / self.wheelbase) * np.tan(inputs[1,0]),
            0.0
        ]]).T

        self.state += dx * dt
        self.state[3,0] = inputs[1,0]

    def get_bicycle_model_visualization_info(self):
        return BicycleModelVisualizationInfo(
            self.state[0,0], 
            self.state[1,0], 
            self.state[2,0], 
            self.state[3,0],
            self.wheelbase)
