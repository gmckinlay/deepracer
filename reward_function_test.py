from reward_function import *
import numpy as np
import matplotlib.pyplot as plt

params = {
    "all_wheels_on_track": True,        # flag to indicate if the agent is on the track
    "x": -0.40380829983121375,                            # agent's x-coordinate in meters
    "y": -4.2632528025380365,                            # agent's y-coordinate in meters
    "closest_objects": [],         # zero-based indices of the two closest objects to the agent's current position of (x, y).
    "closest_waypoints": [229, 0],       # indices of the two nearest waypoints.
    "distance_from_center": 0.0,         # distance in meters from the track center 
    "is_crashed": False,                 # Boolean flag to indicate whether the agent has crashed.
    "is_left_of_center": False,          # Flag to indicate if the agent is on the left side to the track center or not. 
    "is_offtrack": False,                # Boolean flag to indicate whether the agent has gone off track.
    "is_reversed": False,                # flag to indicate if the agent is driving clockwise (True) or counter clockwise (False).
    "heading": -0.0006387014095341131,                      # agent's yaw in degrees
    "objects_distance": [],         # list of the objects' distances in meters between 0 and track_length in relation to the starting line.
    "objects_heading": [],          # list of the objects' headings in degrees between -180 and 180.
    "objects_left_of_center": [], # list of Boolean flags indicating whether elements' objects are left of the center (True) or not (False).
    "objects_location": [], # list of object locations [(x,y), ...].
    "objects_speed": [],            # list of the objects' speeds in meters per second.
    "progress": 0.20383324903957026,                     # percentage of track completed
    "speed": 1.1673640012741089,                        # agent's speed in meters per second (m/s)
    "steering_angle": 1.2913646697998047,               # agent's steering angle in degrees
    "steps": 1,                          # number steps completed
    "track_length": 68.6832042317529,                 # track length in meters.
    "track_width": 10,                  # width of the track
    "waypoints": []        # list of (x,y) as milestones along the track center
}

def load_track():
    # Track Name from Tracks List
    track_name = "penbay_open"
    # Location of tracks folder
    absolute_path = "../"
    # Get waypoints from numpy file
    waypoints = np.load("%s/tracks/%s.npy" % (absolute_path, track_name))

    print("Number of waypoints = " + str(waypoints.shape[0]))
    # Plot waypoints
    for i, point in enumerate(waypoints):
        waypoint = (point[2], point[3])
        plt.scatter(waypoint[0], waypoint[1])
        print("Waypoint " + str(i) + ": " + str(waypoint))
        params['waypoints'].append((waypoint[0], waypoint[1]))

def test_reward_function():
    # forward_point = (3.675445079803467, -4.26334810256958)
    # location = (2.8343610652165157, -4.175601106834468)
    # optimal_heading = math.degrees(math.atan2(forward_point[1] - location[1], forward_point[0] - location[0]))
    # print("optimal heading", optimal_heading)
    # # heading_diff = abs(optimal_heading - heading)
    # # if heading_diff > 180:
    # #     heading_diff = 360 - heading_diff
    # # print("heading_diff", heading_diff)
    # print(max(math.cos(math.radians(abs(optimal_heading))), 0.01))
    print(reward_function(params))

def plot_track():
    plt.show()
    
load_track()
test_reward_function()