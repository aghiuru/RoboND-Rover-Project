import numpy as np

time = 0
pos = None

def select_points(Rover, angle_min, angle_max):
    normalized_angles = Rover.nav_angles * 180/np.pi
    points = (normalized_angles > angle_min) & (normalized_angles < angle_max)
    points_bitmap = np.zeros_like(normalized_angles)
    points_bitmap[points] = 1
    no_points = np.sum(points_bitmap)
    avg_dist = np.sum(points_bitmap * Rover.nav_dists) / no_points
    return (no_points, avg_dist)

# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    Rover.go_forward = 175
    Rover.throttle_set = 1

    print()
    print()
    print("##############################")
    print("time: {}".format(Rover.total_time))
    print("velocity: {}".format(Rover.vel))
    normalized_angles = Rover.nav_angles * 180/np.pi
    avg_angle = np.mean(normalized_angles)
    print("average angle: {}".format(avg_angle))
    avg_dist = np.mean(Rover.nav_dists)
    print("average dist: {}".format(avg_dist))

    (no_points_front, avg_dist_front) = select_points(Rover, -15, 0)
    print(no_points_front)
    print("average dist front: {}".format(avg_dist_front))


    right_points = (normalized_angles < 0)

    wall_right = False
    if (avg_angle > 0):
        print("WALL RIGHT")
        wall_right = True

    wall_ahead = False
    if (avg_dist_front < 10 or Rover.nav_dists.shape[0] == 0):
        print("WALL AHEAD")
        wall_ahead = True

    print(Rover.mode)

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
                #Rover.throttle = Rover.throttle_set
                Rover.steer = -15
            elif (wall_ahead and not wall_right):
                #Rover.throttle = Rover.throttle_set
                Rover.steer = 15
            elif (not wall_ahead and wall_right):
                Rover.steer = 0
                Rover.throttle = .5 #front_points * 1. / 1000  #Rover.throttle_set
            elif (wall_ahead and wall_right):
                #Rover.throttle = Rover.throttle_set
                Rover.steer = -15
            # If there's a lack of navigable terrain pixels then go to 'stop' mode

            if no_points_front < Rover.stop_forward: # or Rover.vel < 0.05:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

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
                if no_points_front < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = 15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if no_points_front >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = 0
                    #Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
    # Just to make the rover do something
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    return Rover
