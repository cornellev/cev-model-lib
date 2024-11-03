import numpy as np
from visualizer import BicycleModelVisualizationInfo

class KinematicBicycleModel:
    """ simplest kinematic bicycle model """

    # NUM_STATE_DIMS = 4
    # NUM_INPUT_DIMS = 2

    def __init__(self, wheelbase):
        self.state = np.zeros((4, 1))
        self.wheelbase = wheelbase

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

class DynamicBicycleModel:
    """ dynamic bicycle model """

    # NUM_STATE_DIMS = 5
    # NUM_INPUT_DIMS = 2

    def __init__(self, wheelbase, mass, static_friction, kinetic_friction):
        self.state = np.zeros((5, 1))

        self.wheelbase = wheelbase
        self.mass = mass

        # TODO: figure out other sources of friction (drag force, damping, rolling resistance)
        # self.drag_coefficient = drag_coefficient
        # self.rho = 1.225
        # self.A = wheelbase**2

        # self.damping = damping
        # self.rolling_resistance = rolling_resistance

        self.g = 9.81
        self.mu_s = static_friction
        self.mu_k = kinetic_friction
    
    def _states(self):
        return self.state[0,0],self.state[1,0],self.state[2,0],self.state[3,0],self.state[4,0]

    def update(self, inputs, dt):
        if inputs.shape != (2 ,1): raise Exception("incorrect size of inputs")
        
        x, y, v, orientation, steering_angle = self._states()
        F, next_steering_angle = inputs[0,0], inputs[1,0]

        friction_force = np.sign(v) * np.choose(np.abs(v) < 0.01, [self.mu_k, self.mu_s]) * self.mass * self.g 
        # drag_force = np.sign(v) * 0.5 * self.A * self.rho * self.drag_coefficient * v**2
        # damping_force = self.damping * v
        # rolling_resistance_force = np.sign(v) * self.rolling_resistance * self.mass * self.g

        dstate = np.array([[
            v * np.cos(orientation),
            v * np.sin(orientation),
            (F - friction_force)/self.mass,
            v * np.tan(steering_angle) / self.wheelbase,
            0.0
        ]]).T

        self.state += dstate * dt
        self.state[4,0] = next_steering_angle
        
    def get_bicycle_model_visualization_info(self):
        return BicycleModelVisualizationInfo(
            self.state[0,0], 
            self.state[1,0], 
            self.state[3,0], 
            self.state[4,0],
            self.wheelbase)


