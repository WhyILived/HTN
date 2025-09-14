#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
import pygame
import math
import json
from tf.transformations import euler_from_quaternion
import heapq
import pyttsx3
import time

# --- Voice ---
last_instruction_time = 0
INSTRUCTION_DELAY = 0
# engine = pyttsx3.init()
# engine.setProperty('volume', 1.0)
# engine.setProperty('rate', 150)
# voices = engine.getProperty('voices')
# engine.setProperty('voice', voices[1].id)

# --- Settings ---
PIXEL_SCALE = 50  # meters -> pixels
SCREEN_W, SCREEN_H = 600, 600
GRID_SIZE = 20
CELL_W = SCREEN_W // GRID_SIZE
CELL_H = SCREEN_H // GRID_SIZE
OBSTACLE_RADIUS = 1  # meters

# --- Global state ---
robot_x, robot_y, robot_yaw = 250, 2510, 0
detected_objects = []
destination_cell = None
path_cells = []
start_cell = []
raw_pose_x, raw_pose_y, raw_pose_yaw = 0, 0, 0

# --- Calibration ---
calibrated = False
origin_x, origin_y = 0, 0       # translation origin in meters
calib_tx, calib_ty = 0, 0       # screen translation in pixels
calib_theta = 0                  # rotation offset

# --- Pygame setup ---
pygame.init()
screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
pygame.display.set_caption("YOLO + Grid Navigation")
clock = pygame.time.Clock()
font = pygame.font.SysFont("Arial", 14)

instruction_pub = rospy.Publisher("instructions", String, queue_size=10)

# --- ROS callbacks ---
def pose_callback(msg):
    global robot_x, robot_y, robot_yaw
    global raw_pose_x, raw_pose_y, raw_pose_yaw

    px = msg.pose.pose.position.x
    py = msg.pose.pose.position.y
    q = msg.pose.pose.orientation
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    raw_pose_x, raw_pose_y, raw_pose_yaw = px, py, yaw

    if not calibrated:
        robot_x = SCREEN_W//2 + px * PIXEL_SCALE
        robot_y = SCREEN_H//2 - py * PIXEL_SCALE
        robot_yaw = yaw
        return

    # Apply calibration transform
    dx = px - origin_x
    dy = py - origin_y
    x_cal = dx*math.cos(-calib_theta) - dy*math.sin(-calib_theta)
    y_cal = dx*math.sin(-calib_theta) + dy*math.cos(-calib_theta)

    robot_x = SCREEN_W//2 + calib_tx + x_cal * PIXEL_SCALE
    robot_y = SCREEN_H//2 + calib_ty - y_cal * PIXEL_SCALE
    robot_yaw = yaw - calib_theta

def objects_callback(msg):
    global detected_objects
    try:
        objs = json.loads(msg.data)
        if isinstance(objs, list):
            detected_objects = objs
    except:
        detected_objects = []

# --- Grid & Pathfinding ---
def cell_from_pos(x, y):
    return x // CELL_W, y // CELL_H

def pos_from_cell(cell):
    cx, cy = cell
    return cx * CELL_W + CELL_W // 2, cy * CELL_H + CELL_H // 2

def neighbors(cell):
    x, y = cell
    nbs = []
    for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
        nx, ny = x+dx, y+dy
        if 0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE:
            nbs.append((nx, ny))
    return nbs

def is_obstacle(cell):
    cx, cy = pos_from_cell(cell)
    cell_x_m = (cx - SCREEN_W//2 - calib_tx)/PIXEL_SCALE + origin_x
    cell_y_m = (SCREEN_H//2 - cy - calib_ty)/PIXEL_SCALE + origin_y
    for obj in detected_objects:
        dist = math.hypot(obj['x'] - cell_x_m, obj['y'] - cell_y_m)
        if dist < OBSTACLE_RADIUS:
            if destination_cell and cell == destination_cell:
                continue
            return True
    return False

def heuristic(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def a_star(start, goal):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]
        for neighbor in neighbors(current):
            if is_obstacle(neighbor):
                continue
            tentative_g = g_score[current] + 1
            if tentative_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    return []

# --- Voice guidance ---
# --- Voice + Bluetooth publishing ---
def give_instructions():
    global last_instruction_time, path_cells, start_cell
    now = time.time()
    if now - last_instruction_time < INSTRUCTION_DELAY:
        return

    if not path_cells or len(path_cells) < 2:
        return

    # Recalculate A* every time instructions are given
    path_cells[:] = a_star(start_cell, destination_cell)

    lookahead_idx = min(2, len(path_cells)-1)
    try:
        target = pos_from_cell(path_cells[lookahead_idx])
    except:
        return
    dx = target[0] - robot_x
    dy = robot_y - target[1]  # screen y inverted

    target_angle = math.atan2(dy, dx)
    angle_diff = (target_angle - robot_yaw + math.pi) % (2*math.pi) - math.pi

    # Decide instruction
    if abs(angle_diff) < math.radians(20):
        spoken_instruction = "Go straight"
        bt_instruction = "straight"
    elif angle_diff > 0:
        spoken_instruction = "Turn left on the spot"
        bt_instruction = "left"
    else:
        spoken_instruction = "Turn right on the spot"
        bt_instruction = "right"

    # Update start cell
    start_cell = tuple(cell_from_pos(int(robot_x), int(robot_y)))

    # Speak instruction
    # engine.say(spoken_instruction)
    # engine.runAndWait()

    # Publish to ROS
    instruction_pub.publish(bt_instruction)
    rospy.loginfo(f"Published instruction: {bt_instruction}")

    last_instruction_time = now


# --- Main ---
def main():
    global destination_cell, path_cells
    global calibrated, origin_x, origin_y, calib_tx, calib_ty, calib_theta
    global start_cell, instruction_pub

    rospy.init_node("pygame_grid_nav")
    rospy.Subscriber("/rtabmap/localization_pose", PoseWithCovarianceStamped, pose_callback)
    rospy.Subscriber("/yolo_objects", String, objects_callback)

    # Publisher for Bluetooth instructions
    instruction_pub = rospy.Publisher("instructions", String, queue_size=10)

    

    rate = rospy.Rate(10)

    running = True
    while running and not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = pygame.mouse.get_pos()
                destination_cell = tuple(cell_from_pos(mx, my))
                start_cell = tuple(cell_from_pos(int(robot_x), int(robot_y)))
                path_cells = a_star(start_cell, destination_cell)
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                # Calibrate: rotate and translate so robot faces north
                origin_x, origin_y = raw_pose_x, raw_pose_y
                desired_north = math.pi/2
                calib_theta = raw_pose_yaw - desired_north
                calib_tx = SCREEN_W//2 - robot_x
                calib_ty = SCREEN_H//2 - robot_y
                calibrated = True

        screen.fill((30, 30, 30))
        pygame.draw.rect(screen, (200, 200, 200), (0, 0, SCREEN_W, SCREEN_H), 2)

        # Draw grid
        for i in range(GRID_SIZE+1):
            pygame.draw.line(screen, (60,60,60), (i*CELL_W,0),(i*CELL_W,SCREEN_H))
            pygame.draw.line(screen, (60,60,60), (0,i*CELL_H),(SCREEN_W,i*CELL_H))

        # Draw robot
        pygame.draw.circle(screen, (0,255,0), (int(robot_x), int(robot_y)), 8)
        line_x = robot_x + 20 * math.cos(robot_yaw)
        line_y = robot_y - 20 * math.sin(robot_yaw)
        pygame.draw.line(screen, (255,0,0), (robot_x, robot_y), (line_x, line_y), 2)

        # Draw obstacles with calibration
        for obj in detected_objects:
            ox, oy = obj['x'], obj['y']
            dx = ox - origin_x
            dy = oy - origin_y
            x_rot = dx*math.cos(-calib_theta) - dy*math.sin(-calib_theta)
            y_rot = dx*math.sin(-calib_theta) + dy*math.cos(-calib_theta)
            map_x_px = SCREEN_W//2 + calib_tx + x_rot*PIXEL_SCALE
            map_y_px = SCREEN_H//2 + calib_ty - y_rot*PIXEL_SCALE
            pygame.draw.circle(screen, (255,255,0), (int(map_x_px), int(map_y_px)), 6)
            screen.blit(font.render(obj['label'], True, (255,255,255)), (int(map_x_px)+8, int(map_y_px)-8))

        # Draw path
        if path_cells:
            points = [pos_from_cell(c) for c in path_cells]
            try:
                pygame.draw.lines(screen, (0,0,255), False, points, 3)
            except:
                pass

        pygame.display.flip()
        give_instructions()
        clock.tick(10)
        rate.sleep()

    pygame.quit()

if __name__ == "__main__":
    main()