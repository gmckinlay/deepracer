import math

MAX_SPEED = 4

def reward_function(params):
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    heading = params['heading']
    progress = params['progress']
    steps = params["steps"]
    speed = params["speed"]
    track_width = params["track_width"]
    distance_from_center = params["distance_from_center"]

    next_point = waypoints[closest_waypoints[1]]
    prev_point = waypoints[closest_waypoints[0]]

    track_direction = math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0])
    direction_diff = abs(track_direction - heading)

    reward = 0.01

    if progress == 100:
        reward = 10000.0
    elif params["all_wheels_on_track"] and steps > 0:
        initial_reward = ((progress / steps) * 100) + (speed ** 2)
    
        reward += initial_reward * rewardHeading(direction_diff)
        reward += initial_reward * rewardSpeed(direction_diff, speed)
        reward += initial_reward * rewardCenterLine(track_width, distance_from_center)
    return float(reward)


def rewardHeading(direction_diff):
    multiplier = 1.0
    if direction_diff > 15:
        multiplier = 0.6
    elif direction_diff > 10:
        multiplier = 0.8
    elif direction_diff > 5:
        multiplier = 0.9
    return multiplier


def rewardSpeed(direction_diff, speed):   
    multiplier = 1.0
    if direction_diff > 5 and speed > MAX_SPEED * 0.7:        
        multiplier = 0.7
    elif direction_diff <= 5 and speed >= MAX_SPEED * 0.7:
        multiplier = 1.2
    return multiplier

def testRewardSpeed():
    print(rewardSpeed(1,1))
    print(rewardSpeed(1,4))
    print(rewardSpeed(10,1))
    print(rewardSpeed(10,4))

def rewardCenterLine(track_width, distance_from_center):
    marker_1 = 0.1 * track_width
    marker_2 = 0.25 * track_width
    marker_3 = 0.5 * track_width
    marker_4 = 0.75 * track_width

    multiplier = 1e-3
    if distance_from_center <= marker_1:
        multiplier = 1.0
    elif distance_from_center <= marker_2:
        multiplier = 0.8
    elif distance_from_center <= marker_3:
        multiplier = 0.7
    elif distance_from_center <= marker_3:
        multiplier = 0.6
    return multiplier