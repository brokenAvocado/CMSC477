import pygame
import math
import numpy as np

# Screen settings
WIDTH, HEIGHT = 600, 600
CENTER = (WIDTH // 2, HEIGHT // 2)
RADIUS = 200

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 200, 0)

# Initialize pygame
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Target vs Pose Angles")
font = pygame.font.SysFont(None, 24)
clock = pygame.time.Clock()

# Sliders
target_angle = 0
pose_angle = 0

slider_rect = pygame.Rect(100, 500, 400, 10)
handle_width = 10

def draw_slider(y, value):
    pygame.draw.rect(screen, BLACK, pygame.Rect(100, y, 400, 4))
    handle_x = int(100 + (value + 180) * (400 / 360)) - handle_width // 2
    pygame.draw.rect(screen, GREEN, pygame.Rect(handle_x, y - 6, handle_width, 16))
    return pygame.Rect(handle_x, y - 6, handle_width, 16)

def angle_to_xy(angle_deg, radius):
    # Flip sign to match clockwise-positive convention
    rad = math.radians(-angle_deg - 90)
    x = CENTER[0] + radius * math.cos(rad)
    y = CENTER[1] + radius * math.sin(rad)
    return (int(x), int(y))

def rotate_to(target, pose):
    """
    Returns direction to rotate from pose to target:
    -1 = CCW, 1 = CW
    """
    # Normalize to [-180, 180]
    diff = (target - pose + 180) % 360 - 180
    return 1 if diff > 0 else -1

def draw_arrow(pose_angle, direction):
    """
    Draw an arrow on the circumference at the pose angle,
    pointing CW or CCW along the circle.
    """
    # Arrow base position (on circle at pose_angle)
    base_angle_rad = math.radians(-pose_angle - 90)
    base_x = CENTER[0] + RADIUS * math.cos(base_angle_rad)
    base_y = CENTER[1] + RADIUS * math.sin(base_angle_rad)

    # Direction of arrow tip (CW or CCW, 90° ahead or behind)
    arrow_direction = pose_angle - direction * 90  # Rotate CW or CCW

    tip_angle_rad = math.radians(-arrow_direction - 90)
    tip_x = base_x + 20 * math.cos(tip_angle_rad)
    tip_y = base_y + 20 * math.sin(tip_angle_rad)

    # Arrowhead wings
    wing_angle1 = math.radians(-arrow_direction - 60)
    wing_angle2 = math.radians(-arrow_direction - 120)

    wing1 = (tip_x - 10 * math.cos(wing_angle1), tip_y - 10 * math.sin(wing_angle1))
    wing2 = (tip_x - 10 * math.cos(wing_angle2), tip_y - 10 * math.sin(wing_angle2))

    # Draw arrow shaft
    pygame.draw.line(screen, (0, 128, 0), (base_x, base_y), (tip_x, tip_y), 3)
    # Draw arrowhead
    pygame.draw.polygon(screen, (0, 128, 0), [(tip_x, tip_y), wing1, wing2])


running = True
dragging_target = dragging_pose = False

while running:
    screen.fill(WHITE)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        elif event.type == pygame.MOUSEBUTTONDOWN:
            if draw_slider(500, target_angle).collidepoint(event.pos):
                dragging_target = True
            elif draw_slider(540, pose_angle).collidepoint(event.pos):
                dragging_pose = True

        elif event.type == pygame.MOUSEBUTTONUP:
            dragging_target = dragging_pose = False

        elif event.type == pygame.MOUSEMOTION:
            if dragging_target:
                rel_x = max(0, min(400, event.pos[0] - 100))
                target_angle = (rel_x / 400) * 360 - 180
            elif dragging_pose:
                rel_x = max(0, min(400, event.pos[0] - 100))
                pose_angle = (rel_x / 400) * 360 - 180

    # Draw circle
    pygame.draw.circle(screen, BLACK, CENTER, RADIUS, 1)

    # Draw angles
    pygame.draw.line(screen, RED, CENTER, angle_to_xy(target_angle, RADIUS), 3)
    pygame.draw.line(screen, BLUE, CENTER, angle_to_xy(pose_angle, RADIUS), 3)

    # Draw sliders
    draw_slider(500, target_angle)
    draw_slider(540, pose_angle)

    # Draw text
    screen.blit(font.render(f"Target: {int(target_angle)}°", True, RED), (100, 470))
    screen.blit(font.render(f"Pose: {int(pose_angle)}°", True, BLUE), (100, 580))

    # Direction calculation
    direction = rotate_to(target_angle, pose_angle)
    draw_arrow(target_angle, direction)

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
