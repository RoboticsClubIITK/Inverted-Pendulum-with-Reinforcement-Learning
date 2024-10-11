import pygame
import math
import serial
import time
import numpy as np
from gym import spaces
import logging

class InvertedPendulumEnv:
    def __init__(self):
        pygame.init()


        self.WIDTH, self.HEIGHT = 800, 600
        self.screen = pygame.display.set_mode((self.WIDTH, self.HEIGHT))
        pygame.display.set_caption("Inverted Pendulum Visualization")

        self.WHITE = (255, 255, 255)
        self.BLACK = (0, 0, 0)
        self.RED = (255, 0, 0)
        self.BLUE = (0, 0, 255)

        self.PENDULUM_LENGTH = 200
        self.CART_WIDTH = 100
        self.CART_HEIGHT = 50
        self.RACK_LENGTH = 600  

        # Serial communication
        self.arduino_port = "COM4"  # Replace with your Arduino port
        self.baud_rate = 115200
        self.ser = serial.Serial(self.arduino_port, self.baud_rate, timeout=1)
        time.sleep(2)

        
        self.action_space = spaces.Box(low=-1, high=1, shape=(1,), dtype=np.float32)
        self.state = None
        self.reward_range = (-10, 10)
        self.clock = pygame.time.Clock()
        
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)

    def center(self):
        self.ser.write(b"10\n")  

    def reset(self):
        # Move pendulum to center (0 position)
        self.ser.write(b"10\n")

        self.ser.write(b'R\n')
        self.state = self._get_state()
        
        return self.state

    def step(self, action):
        action = np.clip(action, self.action_space.low, self.action_space.high)*10
        action = np.clip(action, -10, 10)

    
        # Apply action
        self.ser.write(f"0{action[0]:.2f}\n".encode())  #'0' is voltage control mode
        # Wait for a short time to allow the system to respond
        time.sleep(0.027)
        
        for i in range(5): #try many times (some data was getting accumulated because of missed readings)
            if self.ser.in_waiting:
                try:
                    data_waste = self.ser.readline().decode().strip().split(',')    
                except:
                    pass
        self.ser.write(b'R\n')
        new_state = self._get_state()

        
        done = False
        reward = self._calculate_reward([new_state[0]/5000 , new_state[2]/500,math.cos(math.radians(new_state[1])),math.sin(math.radians(new_state[1])), math.radians(new_state[3])],done)
        
        self.state = new_state
        
        return new_state, reward, done, {}

    def render(self):
        self.screen.fill(self.WHITE)
        self._draw_pendulum(self.state[0], self.state[1])
        
        font = pygame.font.Font(None, 36)
        info_text = font.render(f"Pos: {self.state[0]:.2f}, Angle: {self.state[1]:.2f}", True, self.BLACK)
        speed_text = font.render(f"Speed: {self.state[2]:.2f}, Angular Speed: {self.state[3]:.2f}", True, self.BLACK)
        self.screen.blit(info_text, (10, self.HEIGHT - 70))
        self.screen.blit(speed_text, (10, self.HEIGHT - 40))

        pygame.display.flip()
        self.clock.tick(60)

    def close(self):
        self.ser.close()
        pygame.quit()

    def _get_state(self):
        for i in range(10): #try many times
            if self.ser.in_waiting:
                try:
                    data = self.ser.readline().decode().strip().split(',')
                    if len(data) == 4:                  
                        return list(map(float, data))
                except:
                    pass
            time.sleep(0.001)
        return [0, 0, 0, 0]  # Return a default state if unable to read


    
    def _calculate_reward(self,observation,done):
        x,vx,cos,sin,theta_dot = observation
        reward = 0
        reward += cos
        reward -= 0.001*(theta_dot**2)
        reward -= 0.1*abs(x)
        return reward

    def _draw_pendulum(self, x, angle):
        cart_x = self.WIDTH // 2 + (x / 12000) * self.RACK_LENGTH
        cart_y = self.HEIGHT // 2 + 100

        # Draw the rack
        pygame.draw.line(self.screen, self.BLACK, (self.WIDTH // 2 - self.RACK_LENGTH // 2, cart_y + self.CART_HEIGHT // 2),
                         (self.WIDTH // 2 + self.RACK_LENGTH // 2, cart_y + self.CART_HEIGHT // 2), 5)

        # Draw the cart
        pygame.draw.rect(self.screen, self.BLUE, (cart_x - self.CART_WIDTH // 2, cart_y, self.CART_WIDTH, self.CART_HEIGHT))

        # Draw the pendulum
        end_x = cart_x + self.PENDULUM_LENGTH * math.sin(math.radians(angle))
        end_y = cart_y - self.PENDULUM_LENGTH * math.cos(math.radians(angle))
        pygame.draw.line(self.screen, self.RED, (cart_x, cart_y), (end_x, end_y), 5)
        pygame.draw.circle(self.screen, self.RED, (int(end_x), int(end_y)), 10)

def manual_control(env):
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 36)
    input_box = pygame.Rect(10, 10, 140, 32)
    color_inactive = pygame.Color('lightskyblue3')
    color_active = pygame.Color('dodgerblue2')
    color = color_inactive
    active = False
    text = ''
    mode = '1'  # Start in position control mode

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.MOUSEBUTTONDOWN:
                if input_box.collidepoint(event.pos):
                    active = not active
                else:
                    active = False
                color = color_active if active else color_inactive
            if event.type == pygame.KEYDOWN:
                if active:
                    if event.key == pygame.K_RETURN:
                        env.ser.write(f"{mode}{text}\n".encode())
                        text = ''
                    elif event.key == pygame.K_BACKSPACE:
                        text = text[:-1]
                    else:
                        text += event.unicode
                if event.key == pygame.K_SPACE:
                    mode = '0' if mode == '1' else '1'

        env.screen.fill(env.WHITE)

        # Read and display state
        env.ser.write(b'R\n')
        state = env._get_state()
        env._draw_pendulum(state[0], state[1])
        # Display data
        info_text = font.render(f"Pos: {state[0]:.2f}, Angle: {state[1]:.2f}", True, env.BLACK)
        speed_text = font.render(f"Speed: {state[2]:.2f}, Angular Speed: {state[3]:.2f}", True, env.BLACK)
        env.screen.blit(info_text, (10, env.HEIGHT - 70))
        env.screen.blit(speed_text, (10, env.HEIGHT - 40))

        # Draw input box
        txt_surface = font.render(text, True, color)
        width = max(200, txt_surface.get_width()+10)
        input_box.w = width
        env.screen.blit(txt_surface, (input_box.x+5, input_box.y+5))
        pygame.draw.rect(env.screen, color, input_box, 2)

        # Display current mode
        mode_text = font.render(f"Mode: {'Position' if mode == '1' else 'Voltage'}", True, env.BLACK)
        env.screen.blit(mode_text, (env.WIDTH - 200, 10))

        pygame.display.flip()
        clock.tick(60)

    env.close()


if __name__ == "__main__":
    env = InvertedPendulumEnv()
    manual_control(env)