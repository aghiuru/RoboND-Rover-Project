import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward':

            if (not Rover.wall_ahead and not Rover.wall_right):
                Rover.throttle = .1
                Rover.steer = -15
            elif (Rover.wall_ahead and not Rover.wall_right):
                Rover.throttle = .1
                Rover.steer = 15
            elif (not Rover.wall_ahead and Rover.wall_right):
                Rover.throttle = .2
                Rover.steer = 0
            elif (Rover.wall_ahead and Rover.wall_right):
                Rover.throttle = .1
                Rover.steer = 15

            if Rover.black_wall_right:
                Rover.steer = 15

            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            if Rover.wall_ahead and Rover.wall_front:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        elif Rover.mode == "stuck":
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

                if not Rover.wall_front and not Rover.wall_ahead:
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
