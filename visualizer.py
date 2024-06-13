import pygame
import math

pygame.init()

# Constants
LIDAR_RESOLUTION = 360
MAX_DISTANCE = 3000  # You can adjust this based on the maximum distance expected in your data
SCREEN_SIZE = 800
CENTER = SCREEN_SIZE // 2
BACKGROUND_COLOR = (250, 250, 250)
ROBOT_COLOR = (252, 132, 3)
HIGHLIGHTED_COLOR = (255, 0, 0)
DEFAULT_COLOR = (0, 0, 0)
ROBOT_RADIUS = 15
HIGHLIGHTED_RADIUS = 4
DEFAULT_RADIUS = 3
DATA_FILE_PATH = 'C:\\Users\\josep\\Documents\\Github Repo\\PathPilot\\PathPilot\\data4567.txt'
WAIT_TIME = 50  # milliseconds

highlighted_indices = [
    46,  47,  48,  50,  51,  53,  54, 100, 102, 103, 104, 107, 132, 207,
       209, 211, 212, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234,
       235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248,
       249, 250, 251, 252, 253, 254, 255, 256, 257, 258, 301, 302, 303, 306,
       307, 308, 309, 310, 311, 312, 313, 314, 315, 316, 317, 318, 319, 320,
       321, 322, 323, 324, 325, 326, 327, 328, 329, 330
]

# Functions
def get_data_from_arduino(line):
    return line.strip().split(",")[:-1]

def generate_point_positions(resolution):
    angle = 360 / resolution
    return [
        (math.cos((x + 1) * angle / 180 * math.pi), math.sin((x + 1) * angle / 180 * math.pi))
        for x in range(resolution)
    ]

# Initialize positions
point_positions = generate_point_positions(LIDAR_RESOLUTION)

# Set up the drawing window
screen = pygame.display.set_mode([SCREEN_SIZE, SCREEN_SIZE])

def main():
    with open(DATA_FILE_PATH, 'r') as file:
        lines = file.readlines()

    
    scaling_factor = SCREEN_SIZE / MAX_DISTANCE  # Adjust the factor to fill the screen

    for line in lines:
        distances = get_data_from_arduino(line)
        if len(distances) == LIDAR_RESOLUTION:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return

            screen.fill(BACKGROUND_COLOR)

            for i, distance in enumerate(distances):
                scaled_distance = int(float(distance)) * scaling_factor  # Apply the new scaling factor
                x_pos = point_positions[i][0] * scaled_distance + CENTER
                y_pos = point_positions[i][1] * scaled_distance + CENTER
                color = HIGHLIGHTED_COLOR if i in highlighted_indices else DEFAULT_COLOR
                radius = HIGHLIGHTED_RADIUS if i in highlighted_indices else DEFAULT_RADIUS
                pygame.draw.circle(screen, color, (int(x_pos), int(y_pos)), radius)

            pygame.draw.circle(screen, ROBOT_COLOR, (CENTER, CENTER), ROBOT_RADIUS)
            pygame.display.flip()
            pygame.time.wait(WAIT_TIME)

    pygame.quit()

if __name__ == "__main__":
    main()
