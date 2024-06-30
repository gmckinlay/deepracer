import math

class Reward:
    def __init__(self, verbose=False, look_ahead_alignment=3, look_ahead_speed=10):
        self.verbose = verbose
        self.prev_speed = 0
        self.prev_steering_angle = 0
        self.look_ahead_alignment = look_ahead_alignment
        self.look_ahead_speed = look_ahead_speed

    def print(self, *params):
        if self.verbose:
            print(params)

    def logistic_decreasing_function(self, x, y):
        return 1 / (1 + math.exp((x - y / 2) / (y / 10)))

    def linear_decreasing_function(self, x, y):
        return 1 - (x / y)

    def logarithmic_decreasing_function(self, x, y):    
        return 1 - (math.log(x + 1) / math.log(y + 1))

    def find_next_waypoints(self, waypoints, closest_waypoints, look_ahead):
        next_points = list(range(closest_waypoints[1], closest_waypoints[1] + look_ahead))
        for i in range(len(next_points)):
            if next_points[i] >= len(waypoints):
                next_points[i] -= len(waypoints)        
        return next_points

    def angle_between_points(self, p1, p2, p3):
        v1 = (p1[0] - p2[0], p1[1] - p2[1])
        v2 = (p3[0] - p2[0], p3[1] - p2[1])
        
        angle1 = math.atan2(v1[1], v1[0])
        angle2 = math.atan2(v2[1], v2[0])
        
        angle_diff = angle2 - angle1
        
        angle_deg = math.degrees(angle_diff)
        angle_deg = (angle_deg + 360) % 360
        
        return angle_deg        

    def get_steering_reward(self, waypoints, next_points, heading, steering_angle):
        start_point = waypoints[next_points[0]]
        forward_point = waypoints[next_points[-1]]
        dy = forward_point[1] - start_point[1]
        dx = forward_point[0] - start_point[0]
        optimal_heading = math.degrees(math.atan2(dy, dx))
        heading_diff = abs(optimal_heading - heading)
        if heading_diff > 180:
            heading_diff = 360 - heading_diff

        # Combine alignment and smoothness into a single reward
        steering_diff = abs(steering_angle - self.prev_steering_angle)
        self.prev_steering_angle = steering_angle

        alignment_reward = self.logarithmic_decreasing_function(heading_diff, 180)
        smoothness_reward = self.logarithmic_decreasing_function(steering_diff, 60)
        
        result = 0.5 * alignment_reward + 0.5 * smoothness_reward
        self.print("get_steering_reward", result)
        return result

    def get_speed_reward(self, speed, waypoints, next_points):
        start_point = waypoints[next_points[0]]
        mid_point = waypoints[next_points[len(next_points) // 2]]
        forward_point = waypoints[next_points[-1]]
        curvature = self.angle_between_points(start_point, mid_point, forward_point)

        # Define optimal speeds for different curvature ranges
        if curvature < 10:
            optimal_speed = 3.0  # High speed for straight paths
        elif curvature < 30:
            optimal_speed = 2.5
        else:
            optimal_speed = 1.0  # Low speed for sharper turns

        # Calculate the speed difference
        speed_diff = abs(speed - optimal_speed)

        # Reward function based on the speed difference
        max_speed_diff = 2.0  # Maximum possible difference between speed options
        result = self.logistic_decreasing_function(speed_diff, max_speed_diff)
        
        self.print("Speed Reward: Speed =", speed, "Optimal Speed =", optimal_speed, "Reward =", result)
        return result
    
    def get_center_line_reward(self, track_width, distance_from_center):
        y = track_width / 2
        result = self.logistic_decreasing_function(distance_from_center, y)
        return result

    def get_off_track(self, track_width, distance_from_center):
        return distance_from_center > track_width / 2

    def reward_fun(self, params):        
        steps = params["steps"]
        track_width = params["track_width"]
        distance_from_center = params["distance_from_center"]

        reward = 1e-3
     
        if not self.get_off_track(track_width, distance_from_center) and steps > 0:
            progress = params['progress']
            waypoints = params['waypoints']
            closest_waypoints = params['closest_waypoints']
            total_waypoints = len(waypoints)

            if params['is_reversed']:
                waypoints = list(reversed(waypoints))
                closest_waypoints = [
                    total_waypoints - 1 - params['closest_waypoints'][0],
                    total_waypoints - 1 - params['closest_waypoints'][1]
                ]                
        
            heading = params['heading']
            steering_angle = params['steering_angle']
            speed = params['speed']            

            next_points_steering = self.find_next_waypoints(waypoints, closest_waypoints, self.look_ahead_alignment)
            next_points_speed = self.find_next_waypoints(waypoints, closest_waypoints, self.look_ahead_speed)
            
            center_line_reward = self.get_center_line_reward(track_width, distance_from_center)
            steering_reward = self.get_steering_reward(waypoints, next_points_steering, heading, steering_angle)
            speed_reward = self.get_speed_reward(speed, waypoints, next_points_speed)

            # Combined progress and efficiency reward
            progress_reward = progress / 100
            efficiency_reward = progress / steps
            combined_progress_efficiency_reward = 0.5 * progress_reward + 0.5 * efficiency_reward

            # Combine all rewards
            reward += center_line_reward * 0.3        
            reward += speed_reward * 0.35
            reward += combined_progress_efficiency_reward * 0.3            
            reward += steering_reward * 0.05

            self.print("reward", reward)
        return float(reward)

reward_obj = Reward()

def reward_function(params):
    return reward_obj.reward_fun(params)
