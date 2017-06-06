import numpy as np
import os


def rover_print(s):
    print("ROVER: {}".format(s))


def select_points(Rover, angle_min, angle_max):
    normalized_angles = Rover.nav_angles * 180/np.pi
    points = (normalized_angles > angle_min) & (normalized_angles < angle_max)
    points_bitmap = np.zeros_like(normalized_angles)
    points_bitmap[points] = 1
    no_points = np.sum(points_bitmap)
    avg_dist = np.sum(points_bitmap * Rover.nav_dists) / no_points
    return (no_points, avg_dist)


def angles_mask(dists, angles, angle_min, angle_max):
    normalized_angles = angles * 180/np.pi
    return (normalized_angles > angle_min) & (normalized_angles < angle_max)


def mask(dists, dist_min, dist_max, angles, angle_min, angle_max):
    normalized_angles = angles * 180/np.pi
    return (normalized_angles > angle_min) & \
            (normalized_angles < angle_max) & \
            (dists > dist_min) & \
            (dists < dist_max)

def distance(pos1, pos2):
    x_pixel = pos1[0] - pos2[0]
    y_pixel = pos1[1] - pos2[1]
    return np.sqrt(x_pixel**2 + y_pixel**2)


# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    os.system('clear')
    rover_print("##############################")
    if (Rover.last_pos):
        rover_print("dist: {}".format(distance(Rover.pos, Rover.last_pos[1])))

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

    rover_print("last pos: {}".format(Rover.last_pos))
    rover_print("time: {}".format(Rover.total_time))
    rover_print("velocity: {}".format(Rover.vel))

    normalized_angles = Rover.nav_angles * 180/np.pi
    avg_angle = np.mean(normalized_angles)
    rover_print("average angle: {}".format(avg_angle))

    avg_dist = np.mean(Rover.nav_dists)
    rover_print("average dist: {}".format(avg_dist))

    (no_points_front, avg_dist_front) = select_points(Rover, -25, 0)
    rover_print("#points front: {}".format(no_points_front))
    rover_print("average dist front: {}".format(avg_dist_front))

    right_mask = mask(Rover.nav_dists, 0, 18, Rover.nav_angles, -90, 0)
    right_dists = Rover.nav_dists[right_mask]
#    right_angles = Rover.nav_angles[right_mask]
    no_points_right = len(right_dists)
    rover_print("# POINTS RIGHT: {}".format(no_points_right))

    front_mask = mask(Rover.nav_dists, 0, 15, Rover.nav_angles, -35, 35)
    dists_front = Rover.nav_dists[front_mask]
    no_points_front = len(dists_front)
    rover_print("# POINTS FRONT RIGHT: {}".format(no_points_front))

    very_front_mask = mask(Rover.nav_dists, 0, 10, Rover.nav_angles, -45, 45)
    dists_very_front = Rover.nav_dists[very_front_mask]
    no_points_very_front = len(dists_very_front)
    rover_print("# POINTS IMMMEDIATE FRONT: {}".format(no_points_very_front))

    black_right_mask = mask(Rover.black_dists, 0, 20, Rover.black_angles, -90, -35)
    black_right_dists = Rover.black_dists[black_right_mask]
    no_points_black_right = len(black_right_dists)
    rover_print("# BLACK POINTS RIGHT: {}".format(no_points_black_right))

    black_wall_right = False
    if (no_points_black_right > 1):
        black_wall_right = True

    wall_right = False
    if (no_points_right < 100):
        rover_print("WALL RIGHT")
        wall_right = True

    wall_ahead = False
    if (no_points_right < 10 or \
        no_points_front < 90 or \
        Rover.nav_dists.shape[0] == 0):
        rover_print("WALL AHEAD")
        wall_ahead = True

    wall_front = False
    if (no_points_very_front < 30):
        rover_print("WALL FRONT")
        wall_front = True

    rover_print(Rover.mode)

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward':

            # # Check the extent of navigable terrain
            # if len(Rover.nav_angles) >= 100:
            #     # If mode is forward, navigable terrain looks good
            #     # and velocity is below max, then throttle
            #     if Rover.vel < Rover.max_vel:
            #         # Set throttle value to throttle setting
            #         Rover.throttle = Rover.throttle_set
            #     else: # Else coast
            #         Rover.throttle = 0
            #     Rover.brake = 0
            #     # Set steering to average angle clipped to the range +/- 15
            #     Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)

            if (not wall_ahead and not wall_right):
                Rover.throttle = .1
                Rover.steer = -15
            elif (wall_ahead and not wall_right):
                Rover.throttle = .1
                Rover.steer = 15
            elif (not wall_ahead and wall_right):
                Rover.steer = 0
                #Rover.throttle = .3
                Rover.throttle = .2 # np.clip(float(no_points_front) / 3000, .1, .5)
            elif (wall_ahead and wall_right):
                Rover.throttle = .1
                Rover.steer = 15
            # If there's a lack of navigable terrain pixels then go to 'stop' mode

            if black_wall_right:
                Rover.steer = 15

            if wall_ahead and wall_front: # avg_dist_front < 10: #no_points_front < Rover.stop_forward: # or Rover.vel < 0.05:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        elif Rover.mode == "stuck":
            # if (Rover.stuck_time + .5 < Rover.total_time):
            Rover.throttle = 0
            Rover.brake = 0
            Rover.steer = 15

            if (Rover.stuck_time + 2 < Rover.total_time):
                Rover.throttle = 0
                Rover.brake = 0
                Rover.steer = -15

            if (Rover.stuck_time + 3.3 < Rover.total_time):
                Rover.throttle = -0.3
                Rover.brake = 0
                Rover.steer = 0

            if (Rover.stuck_time + 4 < Rover.total_time):
                Rover.throttle = 0
                Rover.mode = "stop"
                Rover.stuck_time = 0


        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward

                if not wall_front and not wall_ahead:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = 0
                    #Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
                else:
                    Rover.throttle = 0 # [0, -.2][np.random.randint(2)]
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = 15 #[-15, 15][np.random.randint(2)] # Could be more clever here about which way to turn
                 # If we're stopped but see sufficient navigable terrain in front then go!

    # Just to make the rover do something
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    return Rover
