# Throttle setting when accelerating
THROTTLE_SET = 0.4 

# Brake setting when braking
BRAKE_SET = 20 

# Maximum velocity
MAX_VEL = 2 

# If the pitch exceeds this value, then don't update the world map
PITCH_TOLERANCE = 2

# the distance to maintain on the left side of the rover.
SIDE_WALL_DISTANCE = 23

# The wall distance in front at which the rover will stop and start to reverse & turn
FRONT_WALL_STOP_DISTANCE = 25

# when reversing on coming near an obstacle, this is the minimum clearance required till it keeps reversing
REVERSE_MODE_CLEAR_DISTANCE = 20

# when turning away from the obstacle, this is the minimum distance in front that should be clear to go to the next mode
STOP_TURN_RECOVERY_CLEARANCE = 30

# The minimum clear path in front of Rover required to exit the initial recon mode
# measured as 'len(Rover.nav_angles)'
RECON_CLEARANCE_THRESHOLD = 8000

# Distance to set when an object is at a very far distance (arbitraty large value used)
INFINITE_DISTANCE = 1000

# The angle (in degrees) to use for the left wall distance
RANGE_FINDER_ANGLE = 35

# The beam width (in degrees) to measure wall distance
ANGLE_TOLERANCE = 2.5

# Pickup rocks only if the distance is less than this value
ROCK_PICKUP_DISTANCE_THRESHOLD = 45

# As this is a left wall crawler, dont pickup rocks that are on the right side
ROCK_PICKUP_ANGLE_THRESHOLD = -15

# The minimum map that needs to be mapped before ending the run
MAP_COMPLETION_OBJECTIVE_PERC = 90

# The minimum distance this rover needs to be from the start position, to end the run
MAP_COMPLETION_DISTANCE_FROM_START = 4.25

# After perspective transform of the navigable image, clip the top part as its too far ahead
# This top part is not updated in the world map
NAVIGALBE_TRANS_IMAGE_CLIP = 0.25

# The probable obstacles are removed from navigable map
# likely_obs = obstacle >= constants.OBSTACLE_PROBABILITY_FACTOR * navigable
# navigable[likely_obs] = 0
OBSTACLE_PROBABILITY_FACTOR = 3