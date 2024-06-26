import math

class Reward:
    def __init__(self, verbose=False, track_time=False):
        self.prev_speed = 0
        self.prev_steering_angle = 0
        self.steering_alignement_weight = 2
        self.speed_weight = 1.1
        self.steering_smoothness_weight = 1.3
        self.look_ahead = 3

    def find_next_waypoints(self, waypoints, closest_waypoints, look_ahead):
        waypoints = waypoints
        next_points = (list(range(closest_waypoints[1], closest_waypoints[1] + look_ahead)))
        for i in range(len(next_points)):
            if next_points[i] >= len(waypoints):
                next_points[i] -= len(waypoints)        
        return next_points

    def angle_between_points(self, p1, p2, p3):
        # Calculates the angle between two line segments formed by three points
        v1 = (p1[0] - p2[0], p1[1], p2[1])
        v2 = (p3[0] - p2[0], p3[1], p2[1])
        
        angle1 = math.atan2(v1[1], v1[0])
        angle2 = math.atan2(v2[1], v2[0])
        
        angle_diff = angle2 - angle1
        
        angle_deg = math.degrees(angle_diff)
        angle_deg = (angle_deg + 360) % 360
        
        return angle_deg
        

    def get_steering_alignment_reward(self, waypoints, next_points, heading, location):
        # Get Destination coordinates        
        forward_point = waypoints[next_points[len(next_points)-1]]

        #Reward optimal steering
        print("heading", heading)
        print("next_points", next_points)
        print("forward point", forward_point)
        print("location", location)
        optimal_heading = math.degrees(math.atan2(forward_point[1] - location[1], forward_point[0] - location[0]))
        print("optimal heading", optimal_heading)
        heading_diff = abs(optimal_heading - heading)
        print("heading diff", optimal_heading)
        if heading_diff > 180:
            heading_diff = 360 - heading_diff
        # print("heading_diff", heading_diff)
        return max(math.cos(math.radians(heading_diff)), 0.01)
        
    def get_steering_smoothness_reward(self, steering_angle):
        #Reward steering smoothness
        prev_steering_angle = self.prev_steering_angle
        self.prev_steering_angle = steering_angle
        steering_diff = abs(steering_angle - prev_steering_angle)        
        return max(math.exp(-0.5 * steering_diff), 0.01)        

    def get_speed_reward(self, speed, waypoints, next_points, location):
        #Calculate curvature
        next_point = waypoints[next_points[0]]
        forward_point = waypoints[next_points[len(next_points)-1]]
        curvature = self.angle_between_points(location, next_point, forward_point)

        # Optimal speed based on curvature
        min_speed, max_speed = 1, 4
        # Changed to continuous function for optimal speed calculation
        optimal_speed = max_speed - (curvature / 180) * (max_speed - min_speed)

        # Calculate reward for speed
        speed_diff = abs(speed - optimal_speed)
        return max(math.exp(-0.5 * speed_diff), 0.01)

    def get_reward(self, steering_alignement_reward, steering_smoothness_reward, speed_reward):
        return 1 + (steering_alignement_reward * self.steering_alignement_weight) # + (steering_smoothness_reward * self.steering_smoothness_weight) + (speed_reward * self.speed_weight)

    def reward_fun(self, params):
        waypoints = params['waypoints']
        closest_waypoints = params['closest_waypoints']
        total_waypoints = len(waypoints)

        if  params['is_reversed'] :
            waypoints = list(reversed(waypoints))
            closest_waypoints = [
                total_waypoints - 1 - params['closest_waypoints'][0],
                total_waypoints - 1 - params['closest_waypoints'][1]
            ]                
       
        closest_waypoints = params['closest_waypoints']
        heading = params['heading']
        steering_angle = params['steering_angle']
        speed = params['speed']
        location = (params['x'], params['y'])

        next_points = self.find_next_waypoints(waypoints, closest_waypoints, self.look_ahead)
        
        # Steering reward
        steering_alignement_reward = self.get_steering_alignment_reward(waypoints, next_points, heading, location)
        steering_smoothness_reward = self.get_steering_smoothness_reward(steering_angle)
        speed_reward = self.get_speed_reward(speed, waypoints, next_points, location)

        # print("steering_alignement_reward: ", steering_alignement_reward)
        # print("steering_smoothness_reward: ", steering_smoothness_reward)
        # print("speed_reward: ", speed_reward)

        return self.get_reward(steering_alignement_reward, steering_smoothness_reward, speed_reward)

reward_obj = Reward()

def reward_function(params):
    return reward_obj.reward_fun(params)    