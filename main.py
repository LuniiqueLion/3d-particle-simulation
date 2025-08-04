import sys
import random
import pygame
from quadtree import Point3D, Octree, Boundary
from math import cos, sin
import numpy as np

def plot_point(x, y, z, echelle=1):
  points = [[x, y, z]]
  for point in points:
    rotate_x = np.dot(rotation_x, point)
    rotate_y = np.dot(rotation_y, rotate_x)
    rotate_z = np.dot(rotation_z, rotate_y)
    matrice_de_projection = [[1, 0, 0], [0, 1, 0], [0, 0, 0]]
    point_2d = np.dot(matrice_de_projection, rotate_z)

    x = (point_2d[0] * echelle)+ window_width/2
    y = -(point_2d[1] * echelle) + window_height/2
    
  return (int(x), int(y))


# Function to update the positions and velocities of points
def update_positions_and_velocities(node, time_step):
    for point in node.points:
        force_x, force_y, force_z = octree.calculate_force(point)
        acceleration_x = force_x/point.mass  # Assuming mass = 1
        acceleration_y = force_y/point.mass  # Assuming mass = 1
        acceleration_z = force_z/point.mass  # Assuming mass = 1
        point.velocity_x += acceleration_x * time_step
        point.velocity_y += acceleration_y * time_step
        point.velocity_z += acceleration_z * time_step

        point.x += point.velocity_x * time_step
        point.y += point.velocity_y * time_step
        point.z += point.velocity_z * time_step

    if node.subdivided:
        for child in [node.northeast, node.northwest, node.southeast, node.southwest]:
            update_positions_and_velocities(child, time_step)

# Function to collect and return all points in the octree
def collect_all_points(node, points):
    points.extend(node.points)
    if node.subdivided:
        for child in [node.northeast, node.northwest, node.southeast, node.southwest]:
            collect_all_points(child, points)

window_width, window_height = 1000, 800
time_step = 10**5  # Time step for the simulation

# Create an octree with a 3D boundary that covers the entire window
octree_boundary = Boundary(0, 0, 0, window_width/2, window_height/2, window_height/2)

pygame.init()

# Set up the display window
screen = pygame.display.set_mode((window_width, window_height))
pygame.display.set_caption("Octree 3D Simulation")

# Generate 3D points
max_points_per_node = 2500  # Number of points per node
octree = Octree(octree_boundary, max_points_per_node)
num_points = 400000

for _ in range(num_points):
    x = random.uniform(window_width/-4, window_width/2)
    y = random.uniform(window_height/-2, window_height)
    z = random.uniform(window_height/-4,  window_height/2)
    point = Point3D(x, y, z, mass=1)  # Set the mass to 1
    octree.insert(point)

octree.compute_center_of_mass()

angle_x = 1.5
angle_y = -0.75 
angle_z = 0
running = True
echelle=1
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 4:  # Scroll Up
                echelle *= 1.1
            elif event.button == 5:  # Scroll Down
                echelle /= 1.1
    
    
    key_input = pygame.key.get_pressed()
  # rotate the points
    if key_input[pygame.K_LEFT]:
     angle_y -= 0.05
    if key_input[pygame.K_UP]:
     angle_x -= 0.05
    if key_input[pygame.K_RIGHT]:
     angle_y += 0.05
    if key_input[pygame.K_DOWN]:
     angle_x += 0.05
    if key_input[pygame.K_d]:
     angle_z -= 0.05
    if key_input[pygame.K_q]:
     angle_z += 0.05
    if key_input[pygame.K_f]:
    #reset the angles to initial position
      angle_z = 0
      angle_x = 1.5
      angle_y = -0.75
    
    screen.fill((0, 0, 0))  # Clear the screen
    rotation_x = [[1, 0, 0], [0, cos(angle_x), -sin(angle_x)],[0, sin(angle_x), cos(angle_x)]]

    rotation_y = [[cos(angle_y), 0, sin(angle_y)], [0, 1, 0],[-sin(angle_y), 0, cos(angle_y)]]

    rotation_z = [[cos(angle_z), -sin(angle_z), 0],[sin(angle_z), cos(angle_z), 0], [0, 0, 1]]
    update_positions_and_velocities(octree, time_step)  # Update positions and velocities

    # Collect all points in the octree
    all_points = []
    collect_all_points(octree, all_points)

    for point in all_points:
        x,y = plot_point(point.x,point.y,point.z, echelle)
        pygame.draw.circle(screen, (255,255,255), (x, y), 1)
    
    pygame.display.update()

# Quit Pygame
pygame.quit()
sys.exit()
