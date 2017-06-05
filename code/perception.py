import numpy as np
import cv2
import constants
from scipy.spatial import distance

# Identify pixels above the threshold
def color_thresh(img, rgb_thresh_min, rgb_thresh_max):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Threshold with upper and lower bound
    mask = (img[:,:,0] >= rgb_thresh_min[0]) & (img[:,:,1] >= rgb_thresh_min[1]) & (img[:,:,2] >= rgb_thresh_min[2]) \
       & (img[:,:,0] <= rgb_thresh_max[0]) & (img[:,:,1] <= rgb_thresh_max[1]) & (img[:,:,2] <= rgb_thresh_max[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[mask] = 1
    # Return the binary image
    return color_select

# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to apply a rotation to pixel positions
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated

# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    return warped

# Threshold the rocks
def rock_thresh(img, rgb_thresh_min=(125,102,0), rgb_thresh_max=(204,185,78)):
    output = color_thresh(img, rgb_thresh_min, rgb_thresh_max)
    output = cv2.GaussianBlur(output, (3, 3), 0)
    return output

# Threshold the navigabble part of the image
def navigable_thresh(img, clip_top_ratio = 0.45, rgb_thresh_min=(118, 93, 69), rgb_thresh_max=(255,255,255)):
    nav_th_img = color_thresh(img, rgb_thresh_min, rgb_thresh_max)
    top_half = int(clip_top_ratio * img.shape[0])
    nav_th_img[:top_half,:] = 0 # above the horizon its not navigable
    return nav_th_img

# Threshold the obstalce (wall and sky) of the image
def obstacle_thresh(img, clip_top_ratio = 0.45, rgb_thresh_min=(0, 0, 0), rgb_thresh_max=(118,103,120)):
    obs_th_img = color_thresh(img, rgb_thresh_min, rgb_thresh_max)
    top_half = int(clip_top_ratio * img.shape[0])
    obs_th_img[:top_half,:] = 1 # above the horizon its always an obstacle
    return obs_th_img

# Calculate the minimum obstacle distance in front of the rover
def obstacle_distance(obstacle_dists, obstacle_angles, target_angle_degree):
    min_angle = (target_angle_degree - constants.ANGLE_TOLERANCE)/180*np.pi
    max_angle = (target_angle_degree + constants.ANGLE_TOLERANCE)/180*np.pi
    distances = obstacle_dists[(obstacle_angles >= min_angle) & (obstacle_angles <= max_angle)]
    if len(distances) > 0:
        return np.min(distances)
    return constants.INFINITE_DISTANCE

# Calculate the maximum navigable distance in front of the rover
def navigable_distance(obstacle_dists, obstacle_angles, target_angle_degree):
    min_angle = (target_angle_degree - constants.ANGLE_TOLERANCE)/180*np.pi
    max_angle = (target_angle_degree + constants.ANGLE_TOLERANCE)/180*np.pi
    distances = obstacle_dists[(obstacle_angles >= min_angle) & (obstacle_angles <= max_angle)]
    if len(distances) > 0:
        return np.max(distances)
    return 0

def angle_difference(currentPosition, targetPosition, yaw):
    return (np.arctan2(currentPosition[1] - targetPosition[1], currentPosition[0] - targetPosition[0]) * 180 / np.pi - yaw) % 360 - 180

# Measures and calculates the fields of the rover state based on sensor data
def perception_step(Rover):
    
    # Define constants like source and destination points for perspective transform
    img = Rover.img
    dst_size = 5 
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[img.shape[1]/2 - dst_size, img.shape[0] - bottom_offset],
                  [img.shape[1]/2 + dst_size, img.shape[0] - bottom_offset],
                  [img.shape[1]/2 + dst_size, img.shape[0] - 2*dst_size - bottom_offset], 
                  [img.shape[1]/2 - dst_size, img.shape[0] - 2*dst_size - bottom_offset],
                  ])
    world_map_size = 200
    map_scale = 10
    
    # Apply color threshold to identify navigable terrain/obstacles/rock samples
    rock_thresh_image = rock_thresh(img)
    navigable_thresh_image = navigable_thresh(img)
    obstacle_thresh_image = obstacle_thresh(img)
    
    # Apply perspective transform
    obstacle_trans_image = perspect_transform(obstacle_thresh_image, source, destination)
    rock_trans_image = perspect_transform(rock_thresh_image, source, destination)
    navigable_trans_image = perspect_transform(navigable_thresh_image, source, destination)
    
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    Rover.vision_image[:,:,0] = obstacle_trans_image * 255
    Rover.vision_image[:,:,1] = rock_trans_image * 255
    Rover.vision_image[:,:,2] = navigable_trans_image * 255
    
    # Convert map image pixel values to rover-centric coords & to world coordinates
    navigable_x_rover, navigable_y_rover = rover_coords(navigable_trans_image)
    navigable_x_world, navigable_y_world = pix_to_world(navigable_x_rover, navigable_y_rover, Rover.pos[0], Rover.pos[1], Rover.yaw, world_map_size, map_scale)
    
    # Clip the top part of the navigable image. Paths too far ahead in front are ignored
    top_half = int(constants.NAVIGALBE_TRANS_IMAGE_CLIP * img.shape[0])
    clipped_navigable_trans_image = np.copy(navigable_trans_image)
    clipped_navigable_trans_image[:top_half,:] = 0 
    clipped_navigable_x_rover, clipped_navigable_y_rover = rover_coords(clipped_navigable_trans_image)
    clipped_navigable_x_world, clipped_navigable_y_world = pix_to_world(clipped_navigable_x_rover, clipped_navigable_y_rover, Rover.pos[0], Rover.pos[1], Rover.yaw, world_map_size, map_scale)
    
    rock_x_rover, rock_y_rover = rover_coords(rock_trans_image)
    rock_x_world, rock_y_world = pix_to_world(rock_x_rover, rock_y_rover, Rover.pos[0], Rover.pos[1], Rover.yaw, world_map_size, map_scale)
    obstacle_x_rover, obstacle_y_rover = rover_coords(obstacle_trans_image)
    obstacle_x_world, obstacle_y_world = pix_to_world(obstacle_x_rover, obstacle_y_rover, Rover.pos[0], Rover.pos[1], Rover.yaw, world_map_size, map_scale)
    
    # 7) Update Rover worldmap (to be displayed on right side of screen)
    if Rover.pitch <= constants.PITCH_TOLERANCE or Rover.pitch >= (360 - constants.PITCH_TOLERANCE):
        Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        Rover.worldmap[clipped_navigable_y_world, clipped_navigable_x_world, 2] += 1
    
    # Update Rover pixel distances and angles
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(navigable_x_rover, navigable_y_rover)
    Rover.obstacle_dists, Rover.obstacle_angles = to_polar_coords(obstacle_x_rover, obstacle_y_rover)
    Rover.rock_dists, Rover.rock_angles = to_polar_coords(rock_x_rover, rock_y_rover)
    
    # Measure the distance/clearance available in front and sides
    Rover.recon_distance = navigable_distance(Rover.nav_dists, Rover.nav_angles, 0)
    Rover.front_wall_distance = obstacle_distance(Rover.obstacle_dists, Rover.obstacle_angles, 0)
    Rover.wall_distance = obstacle_distance(Rover.obstacle_dists, Rover.obstacle_angles, constants.RANGE_FINDER_ANGLE)
    
    # Calclate distance and angle from start position
    Rover.distance_from_start = distance.euclidean(Rover.pos, Rover.start_pos)
    Rover.angle_to_start = angle_difference(Rover.pos, Rover.start_pos, Rover.yaw)
    
    Rover.rock_size = len(Rover.rock_dists)
    if Rover.rock_size > 0:
        Rover.rock_dist = np.mean(Rover.rock_dists)
        Rover.rock_angle = np.mean(Rover.rock_angles * 180/np.pi)
        Rover.rock_pos = (np.mean(rock_x_world), np.mean(rock_y_world))
    else:
        Rover.rock_dist = 0
        Rover.rock_angle = 0
        Rover.rock_pos = None
    
    return Rover