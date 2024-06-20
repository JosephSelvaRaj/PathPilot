import pygame
import math

pygame.init()

# Constants
LIDAR_RESOLUTION = 240
MAX_DISTANCE = 2000  # You can adjust this based on the maximum distance expected in your data
SCREEN_SIZE = 800
CENTER = SCREEN_SIZE // 2
BACKGROUND_COLOR = (250, 250, 250)
ROBOT_COLOR = (252, 132, 3)
HIGHLIGHTED_COLOR = (255, 0, 0)
DEFAULT_COLOR = (0, 0, 0)
ROBOT_RADIUS = 15
HIGHLIGHTED_RADIUS = 4
DEFAULT_RADIUS = 3
DATA_FILE_PATH = 'C:\\Users\\josep\\Documents\\Github Repo\\PathPilot\\PathPilot\\MASTERDATA240CIRCLE.txt'
WAIT_TIME = 200  # milliseconds

highlighted_indices = [
    136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149,
       150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163,
       164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 176, 184, 185,
       186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199,
       200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213,
       214, 215, 216, 217, 218, 219, 220, 221, 222, 223
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
