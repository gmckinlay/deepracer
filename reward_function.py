import math


class Reward:
    def __init__(self, verbose=False, track_time=False):
        self.prev_speed = 0
        self.prev_steering_angle = 0
        self.steering_alignement_weight = 1.5
        self.speed_weight = 1.1
        self.steering_smoothness_weight = 1.3

    def find_next_three_waypoints(self, waypoints, closest_waypoints):
        waypoints = waypoints
        next_points = (list(range(closest_waypoints[1], closest_waypoints[1] + 3)))
        for i in range(len(next_points)):
            if next_points[i] >= len(waypoints):
                next_points[i] -= len(waypoints)        
        return next_points

    def get_line_points(self, x1, y1, x2, y2, distance=0.1):
        dx = x2 - x1
        dy = y2 - y1
        line_length = math.sqrt(dx ** 2 + dy ** 2)
        num_points = int(line_length / distance) + 1
        x_steps = dx / (num_points - 1)
        y_steps = dy / (num_points - 1)
        line_points = [(x1 + i * x_steps, y1 + i * y_steps) for i in range(num_points)]
        return line_points

    def angle_between_points(self, x, y, b, c):
        """Calculates the angle between two line segments formed by three points."""
        ang = math.degrees(math.atan2(c[1]-b[1], c[0]-b[0]) - math.atan2(y-b[1], x-b[0]))
        return ang + 360 if ang < 0 else ang
        

    def get_steering_alignment_reward(self, waypoints, next_points, heading, x, y):
        # Get Destination coordinates
        b = waypoints[next_points[1]]
        c = waypoints[next_points[2]]

        #Reward optimal steering
        optimal_heading = self.angle_between_points(x, y, b, c)
        heading_diff = abs(optimal_heading - heading)
        if heading_diff > 180:
            heading_diff = 360 - heading_diff
        return max(math.cos(math.radians(heading_diff)), 0.01)
        
    def get_steering_smoothness_reward(self, steering_angle):
        #Reward steering smoothness
        prev_steering_angle = self.prev_steering_angle
        self.prev_steering_angle = steering_angle
        steering_diff = abs(steering_angle - prev_steering_angle)
        return max(math.exp(-0.5 * steering_diff), 0.01)        

    def get_speed_reward(self, speed, waypoints, next_points, x, y):
        #Calculate curvature
        b = waypoints[next_points[1]]
        c = waypoints[next_points[2]]
        curvature = self.angle_between_points(x, y, b, c)        

        # Optimal speed based on curvature
        min_speed, max_speed = 1, 4
        # Changed to continuous function for optimal speed calculation
        optimal_speed = max_speed - (curvature / 180) * (max_speed - min_speed)

        # Calculate reward for speed
        speed_diff = abs(speed - optimal_speed)
        return max(math.exp(-0.5 * speed_diff), 0.01)

    def get_reward(self, steering_alignement_reward, steering_smoothness_reward, speed_reward):
        return 1 + (steering_alignement_reward * self.steering_alignement_weight) + (steering_smoothness_reward * self.steering_smoothness_weight) + (speed_reward * self.speed_weight)

    def reward_fun(self, params):
        waypoints = params['waypoints']
        closest_waypoints = params['closest_waypoints']
        heading = params['heading']
        steering_angle = params['steering_angle']
        speed = params['speed']
        x = params['x']
        y = params['y']

        next_points = self.find_next_three_waypoints(waypoints, closest_waypoints)
        # Steering reward
        steering_alignement_reward = self.get_steering_alignment_reward(waypoints, next_points, heading, x, y)
        steering_smoothness_reward = self.get_steering_smoothness_reward(steering_angle)
        speed_reward = self.get_speed_reward(speed, waypoints, next_points, x, y)

        return self.get_reward(steering_alignement_reward, steering_smoothness_reward, speed_reward)

reward_obj = Reward()

def reward_function(params):
    reward = reward_obj.reward_fun(params)
    return float(reward)    
    
    