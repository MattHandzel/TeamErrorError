import pygame
from time import sleep
import time
import numpy as np
# Initialize a pygame canvas and game
pygame.init()
screen = pygame.display.set_mode((604, 604))
pygame.display.set_caption("Draw Robot Trajectory")
clock = pygame.time.Clock()

BLACK = (0, 0, 0)
WHITE = (255, 255, 255)

def blitRotateCenter(surf, image, topleft, angle):

  rotated_image = pygame.transform.rotate(image, angle)
  new_rect = rotated_image.get_rect(center = image.get_rect(topleft = topleft).center)

  surf.blit(rotated_image, new_rect)

imp = pygame.image.load("./GameFieldDisks.png").convert()

pixel_to_cm = (12 * 2.54 * 12) / (594 - 11) 

# Using blit to copy content from one surface to other
screen.blit(imp, (0, 0))

# paint screen one time
pygame.display.flip()

current_trajectory = []

done = False

current_rotation = 0
previous_rotation_time = 0 

while not done:
  # When a player clicks on a position on the screen, draw a pixel
  
  if pygame.mouse.get_pressed()[0] == 1:
      # Draw a rectangle at the position
      pos = pygame.mouse.get_pos()
      current_trajectory.append(np.array([pos[0], -pos[1]]) * pixel_to_cm)
      pygame.draw.rect(screen, BLACK, (pos[0], pos[1], 10, 10))
  
  pygame.display.update()
  clock.tick(60)

  if time.time() - previous_rotation_time > 0.5:
    if pygame.key.get_pressed()[pygame.K_q]:
      current_rotation += 90
      blitRotateCenter(screen, imp, (0, 0), current_rotation)
      previous_rotation_time = time.time()
    elif pygame.key.get_pressed()[pygame.K_e]:
      current_rotation -= 90
      blitRotateCenter(screen, imp, (0, 0), current_rotation)
      previous_rotation_time = time.time()
    
    

  for event in pygame.event.get():
    if event.type == pygame.QUIT:
      pygame.quit()
      done = True
    

  # Update the screen

# if there is no trajectory, exit
if len(current_trajectory) == 0:
  exit()

# Save the trajectory in a file 

current_trajectory = np.array(current_trajectory)
print("Before:\t", len(current_trajectory))
previous_point = np.array([None, None])

copy_of_trajectory = current_trajectory.copy()
num_del = 0
for i, point in enumerate(copy_of_trajectory):
  if previous_point.any() == None:
    previous_point = point
    continue

  # If the current point is too close to the previous point, remove the previous point
  if (((point[0] - previous_point[0]) ** 2 + abs(point[1] - previous_point[1]) ** 2) ** 0.5) < 1:
    current_trajectory = np.delete(current_trajectory, i - num_del, axis=0)
    num_del += 1
  else:
    previous_point = point

print("After:\t", len(current_trajectory))


current_trajectory -= current_trajectory[0]

nameOfTrajectory = input("Filename:\t")

nameOfTrajectory = nameOfTrajectory if nameOfTrajectory != "" else f"{round(time.time())}_trajectory"

with open(f"{nameOfTrajectory}.txt", "w") as f:
  for point in current_trajectory:
    f.write(
      '''{ 'x' : ''' + str(round(point[0] * 100) / 100) + ''', 'y' : ''' + str(round(point[1] * 100) / 100) + '''},'''
    )