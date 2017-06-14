import numpy as np
import constants
import perception
from scipy.spatial import distance

def decision_step(Rover):
    
    if Rover.mode == 'start':
        Rover.start_pos = Rover.pos
        Rover.mode = 'initial_recon'
        return Rover
        
    if Rover.mode == 'stop_before_end':
        Rover.steer = 0
        Rover.brake = Rover.brake_set
        Rover.throttle = 0
        if Rover.vel == 0:
            Rover.mode = 'end'
        return Rover
        
    if Rover.mode == 'end':
        Rover.steer = np.clip(Rover.angle_to_start, -15, 15) # Steer towards starting position
        if abs(Rover.steer) < 2 :
            Rover.throttle = 0.2
            Rover.brake = 0
        else:
            Rover.brake = Rover.brake_set
        
        if Rover.distance_from_start < 1: # 1 metre radius from starting position --> success!
            Rover.brake = Rover.brake_set
            Rover.throttle = 0
            Rover.steer = 0
            print('Completed the run')
        else:
            Rover.brake = 0
        return Rover
        
    if Rover.mode == 'cruise' and Rover.distance_from_start <= constants.MAP_COMPLETION_DISTANCE_FROM_START and Rover.perc_mapped > constants.MAP_COMPLETION_OBJECTIVE_PERC:
        Rover.brake = Rover.brake_set
        Rover.mode = 'stop_before_end'
        return Rover
    
    if Rover.mode not in ['pickup', 'rotate_before_pickup', 'wait_for_pickup_start', 'wait_for_pickup_end']:
        if Rover.rock_size > 0 and Rover.rock_angle > constants.ROCK_PICKUP_ANGLE_THRESHOLD and Rover.rock_dist < constants.ROCK_PICKUP_DISTANCE_THRESHOLD:
                if not Rover.send_pickup and not Rover.picking_up:
                    Rover.mode = 'rotate_before_pickup'
                    Rover.target_rock_pos = Rover.rock_pos
                    return Rover
                
    if Rover.mode == 'wait_for_pickup_start':
        if Rover.picking_up:
            Rover.mode = 'wait_for_pickup_end'
        return Rover
        
    if Rover.mode == 'pickup':
        distance_to_rock = distance.euclidean(Rover.pos, Rover.target_rock_pos)
        if distance_to_rock < 2 and Rover.rock_size == 0:
                Rover.mode = 'cruise'
                Rover.target_rock_pos = None
                return Rover
        if Rover.near_sample == 1:
            Rover.steer = 0
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            if Rover.vel == 0:
                if Rover.vel == 0 and not Rover.picking_up and not Rover.send_pickup:
                    Rover.send_pickup = True
                    Rover.mode = 'wait_for_pickup_start'
                    return Rover
        else:
            if len(Rover.rock_angles) > 0:
                Rover.steer = np.clip(Rover.rock_angle, -15, 15)
            else:
                Rover.steer = 0
            if Rover.vel < 0.5:
                Rover.throttle = 0.3
                Rover.brake = 0
            elif Rover.vel > 1:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
            else:
                Rover.throttle = 0.2
                Rover.brake = 0
        return Rover
        
    if Rover.mode == 'rotate_before_pickup':
        Rover.throttle = 0
        # Brake as soon as a rock is found
        if Rover.vel > 0:
            Rover.brake = Rover.brake_set
            return Rover
        else:
            Rover.brake = 0
            
        diff_angle = perception.angle_difference(Rover.pos, Rover.target_rock_pos, Rover.yaw)
        if abs(diff_angle) < 2:
            Rover.steer = 0
            Rover.mode = 'pickup'
        else:
            Rover.steer = np.clip(diff_angle, -15, 15)
        return Rover
    
    if Rover.mode == 'wait_for_pickup_end':
        Rover.send_pickup = False
        Rover.brake = Rover.brake_set
        Rover.steer = 0
        Rover.throttle = 0
        if not Rover.picking_up:
            Rover.brake = 0
            Rover.mode = 'cruise'
            Rover.target_rock_pos = None
        return Rover
    
    if Rover.mode == 'initial_recon':
        Rover.steer = 15
        Rover.throttle = 0
        Rover.brake = 0
        if Rover.recon_distance > 159 and len(Rover.nav_angles) > constants.RECON_CLEARANCE_THRESHOLD:
            Rover.brake = Rover.brake_set
            Rover.steer = 0
            Rover.mode = 'cruise'
        
    if Rover.mode == 'cruise':
        Rover.brake = 0
        Rover.steer = 0
        if Rover.front_wall_distance >= constants.FRONT_WALL_STOP_DISTANCE:  
            # If navigable terrain looks good 
            # and velocity is below max, then throttle 
            max_vel = Rover.max_vel
            max_throttle = Rover.throttle_set
            # Halve the max speed if there is a rock nearby
            if Rover.rock_size > 0 :
                max_vel = Rover.max_vel / 2
            if Rover.front_wall_distance < 20:
                max_vel = Rover.max_vel / 2
                max_throttle = Rover.throttle_set / 2
                
            if Rover.vel < max_vel:
                # Set throttle value to throttle setting
                Rover.throttle = max_throttle
            else: # Else coast
                Rover.throttle = 0
            Rover.brake = 0
        elif Rover.front_wall_distance < constants.FRONT_WALL_STOP_DISTANCE:
                # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                if Rover.vel == 0:
                    Rover.mode = 'reverse'
                
        if Rover.wall_distance < constants.SIDE_WALL_DISTANCE:
            Rover.throttle = 0
            Rover.steer = np.clip(Rover.steer-15, -15, 15) #Steer right
        elif Rover.wall_distance < 55:
            Rover.steer = np.clip(Rover.steer+10, -15, 15) #Steer left
        else:
            Rover.steer = 10
            
    if Rover.mode == 'reverse':
        Rover.throttle = -1 * Rover.throttle_set
        Rover.steer = 15
        Rover.brake = 0
        if Rover.front_wall_distance >= constants.REVERSE_MODE_CLEAR_DISTANCE:
            Rover.steer = 0
            Rover.brake = Rover.brake_set
            Rover.throttle = 0
            Rover.mode = 'stop_and_turn'
            
    if Rover.mode == 'stop_and_turn':
        if Rover.vel != 0:
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            Rover.steer = 0
        else:
            Rover.steer = -15
            Rover.brake = 0
            Rover.throttle = 0
            
        if Rover.front_wall_distance > constants.STOP_TURN_RECOVERY_CLEARANCE:
            Rover.steer = 0
            Rover.brake = Rover.brake_set
            Rover.throttle = 0
            Rover.mode = 'cruise'
            
    return Rover