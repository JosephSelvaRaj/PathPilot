import pygame
import math

pygame.init()

# Constants
LIDAR_RESOLUTION = 240
VISUALIZATION_RESOLUTION = 240

def get_data_from_arduino(line):
    return line.strip().split(",")[:-1]

def generate_line_positions(number_of_lines):
    angle_increment = 360 / number_of_lines
    return [
        (
            300 * math.cos(math.radians(angle_increment * i)),
            300 * math.sin(math.radians(angle_increment * i))
        ) for i in range(number_of_lines)
    ]

# Generate positions for visualization lines
line_positions = generate_line_positions(VISUALIZATION_RESOLUTION)

# Set up the drawing window
screen = pygame.display.set_mode([800, 800])
font = pygame.font.SysFont(pygame.font.get_default_font(), 72)

# Read data from file
with open('C:\\Users\\josep\\Documents\\Github Repo\\PathPilot\\PathPilot\\DATARIGHT1.TXT', 'r') as file:
    lines = file.readlines()

# Main loop to process data and visualize
for line in lines:
    distances = get_data_from_arduino(line)
    
    if len(distances) == LIDAR_RESOLUTION:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()
        
        screen.fill((250, 250, 250))

        for i, distance in enumerate(distances):
            distance = max(float(distance), 20) / 2000
            x, y = line_positions[i]
            pos = (x * distance + 400, y * distance + 400)
            color = (255, 0, 0) if 135 <= i <= 227 or 185 <= i <= 227 else (0, 0, 0)
            radius = 3 if color == (255, 0, 0) else 2
            pygame.draw.circle(screen, color, pos, radius)
        
        pygame.draw.circle(screen, (252, 132, 3), (400, 400), 12)
        pygame.display.flip()
        pygame.time.wait(200)

pygame.quit()
