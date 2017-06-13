import numpy as np
import cv2
import os

def rover_print(s):
    print("ROVER: {}".format(s))

def mask(dists, dist_min, dist_max, angles, angle_min, angle_max):
    normalized_angles = angles * 180/np.pi
    return (normalized_angles > angle_min) & \
            (normalized_angles < angle_max) & \
            (dists > dist_min) & \
            (dists < dist_max)

def no_points_mask(dists, dist_min, dist_max, angles, angle_min, angle_max):
    masked = mask(dists, dist_min, dist_max, angles, angle_min, angle_max)
    return len(dists[masked])

def distance(pos1, pos2):
    x_pixel = pos1[0] - pos2[0]
    y_pixel = pos1[1] - pos2[1]
    return np.sqrt(x_pixel**2 + y_pixel**2)

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, lower_rgb_thresh=(150, 150, 150), upper_rgb_thresh=(255, 255, 255)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = ((img[:,:,0] >= lower_rgb_thresh[0]) \
                | (img[:,:,1] >= lower_rgb_thresh[1]) \
                | (img[:,:,2] >= lower_rgb_thresh[2])) & \
                ((img[:,:,0] < upper_rgb_thresh[0]) \
                | (img[:,:,1] < upper_rgb_thresh[1]) \
                | (img[:,:,2] < upper_rgb_thresh[2]))
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

def terrain_thresh(img):
    return color_thresh(img)

def obstacle_thresh(img):
    return color_thresh(img, (1, 1, 1), (150, 150, 150))

def rock_thresh(img):
    return color_thresh(img, (160, 160, 0), (256, 256, 100))

def black_thresh(img):
    return color_thresh(img, (1, 1, 1), (50, 50, 50))

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
    # Apply a rotation
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

# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()

    rover_xpos = Rover.pos[0]
    rover_ypos = Rover.pos[1]
    rover_yaw = Rover.yaw
    worldmap = Rover.worldmap
    scale = 10

    # NOTE: camera image is coming to you in Rover.img
    image = Rover.img

    # 1) Define source and destination points for perspective transform
    dst_size = 5
    # Set a bottom offset to account for the fact that the bottom of the image
    # is not the position of the rover but a bit in front of it
    # this is just a rough guess, feel free to change it!
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                      [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                      [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                      [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                      ])

    # 2) Apply perspective transform
    warped = perspect_transform(image, source, destination)

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples

    ones = np.ones_like(image[:,:,0])
    warped_ones = perspect_transform(ones, source, destination)

    terrain = terrain_thresh(warped)
    #obstacle = obstacle_thresh(warped)
    obstacle = warped_ones - terrain
    rock = rock_thresh(warped)
    black = black_thresh(warped)

    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:,:,0] = obstacle * 255
    Rover.vision_image[:,:,1] = rock * 255
    Rover.vision_image[:,:,2] = terrain * 255

    # 5) Convert map image pixel values to rover-centric coords
    xpix_terrain, ypix_terrain = rover_coords(terrain)
    xpix_obstacle, ypix_obstacle = rover_coords(obstacle)
    xpix_rock, ypix_rock = rover_coords(rock)
    xpix_black, ypix_black = rover_coords(black)

    # 6) Convert rover-centric pixel values to world coordinates
    terrain_x_world, terrain_y_world = pix_to_world(xpix_terrain, ypix_terrain, rover_xpos,
                                    rover_ypos, rover_yaw,
                                    worldmap.shape[0], scale)
    obstacle_x_world, obstacle_y_world = pix_to_world(xpix_obstacle, ypix_obstacle, rover_xpos,
                                    rover_ypos, rover_yaw,
                                    worldmap.shape[0], scale)
    rock_x_world, rock_y_world = pix_to_world(xpix_rock, ypix_rock, rover_xpos,
                                    rover_ypos, rover_yaw,
                                    worldmap.shape[0], scale)

    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    if (Rover.pitch < .01 or Rover.pitch > 359.99 or Rover.roll < .01 or Rover.roll > 359.99):
        Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        Rover.worldmap[terrain_y_world, terrain_x_world, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(xpix_terrain, ypix_terrain)
    Rover.black_dists, Rover.black_angles = to_polar_coords(xpix_black, ypix_black)

    os.system('clear')

    rover_print("Time: {}".format(Rover.total_time))
    rover_print("Last position: {}".format(Rover.last_pos))
    rover_print("Velocity: {}".format(Rover.vel))
    if (Rover.last_pos):
        rover_print("Distance from last position: {}".format(distance(Rover.pos, Rover.last_pos[1])))

    # Tracks positions so that it makes sure it's not stuck.
    if (Rover.last_pos is None):
        Rover.last_pos = (Rover.total_time, Rover.pos)
    else:
        last_time = Rover.last_pos[0]
        current_time = Rover.total_time
        if (last_time + 8 < current_time):
            if (distance(Rover.pos, Rover.last_pos[1]) < .2):
                Rover.mode = "stuck"
                Rover.stuck_time = current_time
            Rover.last_pos = (Rover.total_time, Rover.pos)

    Rover.go_forward = 175
    Rover.throttle_set = 1

    normalized_angles = Rover.nav_angles * 180/np.pi
    avg_angle = np.mean(normalized_angles)
    rover_print("Avg. angle: {}".format(avg_angle))

    avg_dist = np.mean(Rover.nav_dists)
    rover_print("Avg. dist: {}".format(avg_dist))

#    no_points_front = no_points_mask(Rover.nav_dists, 0, 250, Rover.nav_angles, -25, 0)

    no_points_right = no_points_mask(Rover.nav_dists, 0, 18, Rover.nav_angles, -90, 0)
    rover_print("# POINTS RIGHT: {}".format(no_points_right))

    no_points_front = no_points_mask(Rover.nav_dists, 0, 15, Rover.nav_angles, -35, 35)
    rover_print("# POINTS FRONT RIGHT: {}".format(no_points_front))

    no_points_very_front = no_points_mask(Rover.nav_dists, 0, 10, Rover.nav_angles, -45, 45)
    rover_print("# POINTS IMMMEDIATE FRONT: {}".format(no_points_very_front))

    no_points_black_right = no_points_mask(Rover.black_dists, 0, 20, Rover.black_angles, -90, -35)
    rover_print("# BLACK POINTS RIGHT: {}".format(no_points_black_right))

    Rover.black_wall_right = False
    if (no_points_black_right > 1):
        Rover.black_wall_right = True

    Rover.wall_right = False
    if (no_points_right < 100):
        rover_print("WALL RIGHT")
        Rover.wall_right = True

    Rover.wall_ahead = False
    if (no_points_right < 10 or \
        no_points_front < 90 or \
        Rover.nav_dists.shape[0] == 0):
        rover_print("WALL AHEAD")
        Rover.wall_ahead = True

    Rover.wall_front = False
    if (no_points_very_front < 30):
        rover_print("WALL FRONT")
        Rover.wall_front = True

    return Rover
