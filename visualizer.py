import pygame
import numpy as np

class PygameContext:
    def __init__(self, context_size, screen_size):
        self.context_size = context_size
        self.screen_size = screen_size

        self.scale_x = self.screen_size[0] / self.context_size[0]
        self.scale_y = self.screen_size[1] / self.context_size[1]

    def to_pygame(self, ctx_coords):
        return (ctx_coords[0]*self.scale_x + self.screen_size[0]/2,  self.screen_size[1]/2 - self.scale_y * ctx_coords[1])

class PygameEngine:
    def __init__(self, width, height, title="Pygame Window"):
        pygame.init()
        self.width = width
        self.height = height
        self.title = title
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption(self.title)
        self.clock = pygame.time.Clock()
        self.running = True

        self.visualizers = []
        self.input_devices = []

        self.context = PygameContext((2, 2), (self.width, self.height))

    def add_visualizers(self, visualizers):
        self.visualizers += visualizers

    def add_input_devices(self, input_devices):
        self.input_devices += input_devices

    def _handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

        keys = pygame.key.get_pressed()
        for input_device in self.input_devices:
            input_device.update_with_keys(keys)

    def _draw(self):
        for visualizer in self.visualizers:
            visualizer.draw_bicycle(self)

    def run(self, fn=None):
        while self.running:
            self._handle_events()
            self.screen.fill((255, 255, 255))
            self._draw()
            if fn: fn()
            pygame.display.flip()
            self.clock.tick(60.0)

        pygame.quit()

class BicycleModelVisualizationInfo:
    def __init__(self, x, y, yaw, steering_angle, wheel_base):
        self.x = x # rear axis x
        self.y = y # rear axis y
        self.yaw = yaw
        self.steering_angle = steering_angle
        self.wheelbase = wheel_base

class BicycleModelVisualizer:

    def __init__(self, model):
        # TODO: parameterize if and how you visaulize the front wheel
        self.model = model

    def draw_bicycle(self, engine):
        screen = engine.screen
        ctx = engine.context 
        info = self.model.get_bicycle_model_visualization_info()

        # Calculate the position of the front axle
        front_x = info.x + info.wheelbase * np.cos(info.yaw)
        front_y = info.y + info.wheelbase * np.sin(info.yaw)

        # Calculate the front wheel angle
        front_wheel_x = front_x + (info.wheelbase/4) * np.cos(info.yaw + info.steering_angle)
        front_wheel_y = front_y + (info.wheelbase/4) * np.sin(info.yaw + info.steering_angle)

        # Draw the bicycle body
        pygame.draw.line(screen, (0, 0, 0), ctx.to_pygame((info.x, info.y)), ctx.to_pygame((front_x, front_y)), 2)

        # Draw the front wheel (line to indicate steering)
        pygame.draw.line(screen, (255, 0, 0), ctx.to_pygame((front_x, front_y)), ctx.to_pygame((front_wheel_x, front_wheel_y)), 2)



