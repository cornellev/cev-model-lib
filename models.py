import numpy as np
from visualizer import BicycleModelVisualizationInfo
import dataclasses

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

    def __init__(self, wheelbase, mass, static_friction):
        self.state = np.zeros((5, 1))

        self.wheelbase = wheelbase
        self.mass = mass

        # TODO: figure out other sources of friction (drag force, damping, rolling resistance)
        # self.drag_coefficient = drag_coefficient
        # self.rho = 1.225
        # self.A = wheelbase**2

        # self.damping = damping
        # self.rolling_resistance = rolling_resistance

        # NOTE: rolling without slipping is assumed, so just static friction is applied 
        self.g = 9.81
        self.mu_s = static_friction
        # self.mu_k = kinetic friction
    
    def _states(self):
        return self.state[0,0],self.state[1,0],self.state[2,0],self.state[3,0],self.state[4,0]

    def update(self, inputs, dt):
        if inputs.shape != (2 ,1): raise Exception("incorrect size of inputs")
        
        x, y, v, orientation, steering_angle = self._states()
        F, next_steering_angle = inputs[0,0], inputs[1,0]

        # TODO: save friction for higher velocity (oscillates around low velocities)
        friction_force = np.sign(v) * self.mu_s * self.mass * self.g 
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
    
@dataclasses.dataclass
class VoltageBicycleModelMotor:
    N = 19  # Gear reduction ratio
    K_t = 0.215  # Torque constant (Nm/A)
    K_e = 0.12   # Back EMF constant (V/(rad/s))
    R = 2.0      # Motor resistance (Ohms)

    def get_torque(self, V, omega):
        # TODO: implement current limiting as this can "disobey" the stall current
        I = (V - self.K_e * omega) / self.R
        tau = self.K_t * I
        tau_output = self.N * tau
        return tau_output


class VoltageBicycleModel:
    
    # NUM_INPUT_DIM = 2
    # NUM_STATE_DIM = 5

    def __init__(self, wheelbase, wheel_radius, mass, static_friction, motor):
        # [ x, y, orientation, velocity, steering_angle ]
        self.state = np.zeros((5, 1))
        
        self.wheelbase = wheelbase
        self.wheel_radius = wheel_radius
        self.g = 9.81
        self.mass = mass
        self.mu_s = static_friction

        self.motor = motor

        self.servo_rads_per_s_at_4_8_V = 6.16  # rad/s at 4.8V
        self.servo_rads_per_s_at_6_0_V = np.deg2rad(60 / 0.13) # rad/s

    def _states(self):
        for i in range(int(self.state.shape[0])):
            yield self.state[i, 0]

    def update(self, inputs, dt):
        x, y, orientation, velocity, steering_angle = self._states()
        V, target_steering_angle = inputs[0,0], inputs[1,0]

        # compute forces
        wheel_angular_velocity = velocity / self.wheel_radius 
        motor_torque = self.motor.get_torque(V, wheel_angular_velocity)
        F_wheel = motor_torque / self.wheel_radius
        F_friction = np.sign(velocity) * self.mu_s * self.mass * self.g

        # Compute next steering angle
        # TODO: this formula works, maybe a little sketch, but has dt multiplied in already so I divide it out.
        # Maybe I should get it where dt is not already multiplied in 
        angle_diff = target_steering_angle - steering_angle
        max_step = self.servo_rads_per_s_at_6_0_V * dt

        scale = min(1, np.abs(angle_diff)/max_step)
        dsteering_angle = max_step * scale * np.sign(angle_diff) 

        dstate = np.array([[
            np.cos(orientation) * velocity,
            np.sin(orientation) * velocity,
            velocity * np.tan(steering_angle) / self.wheelbase,
            (F_wheel - F_friction) / self.mass,
            dsteering_angle / dt
        ]]).T

        self.state += dstate * dt

    def get_bicycle_model_visualization_info(self):
        return BicycleModelVisualizationInfo(
            self.state[0,0], 
            self.state[1,0], 
            self.state[2,0], 
            self.state[4,0],
            self.wheelbase)


        



