"""
3006ICT Group Project - Controller

Mission:
    Search the observation stations, identify the requested visual target,
    navigate safely, and stop at the correct target.
"""

import json
import math
from pathlib import Path

import cv2
import numpy as np
from controller import Robot

from path_planner import PathPlanner
from project_utils import CONFIG, ROOT, world_to_grid, grid_to_world
from vision import identify_target


# ------------------------------------------------------------------
# Webots setup
# ------------------------------------------------------------------
robot = Robot()
timestep = int(robot.getBasicTimeStep())



left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
camera = robot.getDevice("camera")
ps = [robot.getDevice(f"ps{i}") for i in range(8)]
gps = robot.getDevice("gps")
imu = robot.getDevice("imu")

left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

camera.enable(timestep)
gps.enable(timestep)
imu.enable(timestep)
for sensor in ps:
    sensor.enable(timestep)

# movement control setup
K_TURN = 2.0
MAX_SPEED = 6.28


GRID = np.load(ROOT / "maps" / "occupancy_grid.npy")
MISSION = json.loads((ROOT / "config" / "assessment_mission.json").read_text())
target = MISSION["target"]



# ------------------------------------------------------------------
# Provided low-level helpers
# ------------------------------------------------------------------
def set_speed(left, right):
    left = np.clip(left, -MAX_SPEED, MAX_SPEED)
    right = np.clip(right, -MAX_SPEED, MAX_SPEED)
    left_motor.setVelocity(float(left))
    right_motor.setVelocity(float(right))

def has_arrived(pose, target, threshold = 0.1):
    """ Without this function the robot just literally won't stop moving """
    pose_x, pose_y, _ = pose
    target_x, target_y = target
    distance = math.hypot(target_x - pose_x, target_y - pose_y)
    return distance < threshold

def get_pose():
    """Return provided ground-truth-like pose (x, y, yaw)."""
    x, y, _ = gps.getValues()
    yaw = imu.getRollPitchYaw()[2]
    return x, y, yaw


def camera_bgr():
    h, w = camera.getHeight(), camera.getWidth()
    image = np.frombuffer(camera.getImage(), np.uint8).reshape(h, w, 4)
    return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)


def proximity_values():
    return [sensor.getValue() for sensor in ps]

def move_towards_point(target, pose):
    """Target must be an x y position on the map and pose is the triple (x, y, yaw)"""
    target_x, target_y = target
    pose_x, pose_y, yaw = pose
    
    # Get the target angle 
    target_angle = math.atan2(target_y - pose_y, target_x - pose_x)
    # How off the yaw actually is
    heading_error = target_angle - yaw
    # (heading_error = target_angle - yaw) caused 350 degrees instead of -10 degrees
    heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
    
    # Turning speed
    turn_speed = K_TURN * heading_error
    forward_speed = MAX_SPEED * math.cos(heading_error)
    
    # Converting the forward speed and the turning speed into diferential drive speeds
    left_wheel_speed = forward_speed - turn_speed
    right_wheel_speed = forward_speed + turn_speed
    
    return (left_wheel_speed, right_wheel_speed)
    

# ------------------------------------------------------------------
# Group implementation
# ------------------------------------------------------------------
# TO DO
# Thinking of having these as the possible states the robot will be in, tuple for efficiency. 
STATE = ("TRAVEL_TO_STATION", "ORIENT_TOWARD_STATION", "INSPECTING_STATION", "TRAVEL_TO_TARGET", "DONE")


# Path planner setup
# Have to do one manual timestep to makesure that getpose works
robot.step(timestep)
initial_pose = get_pose()
planner = PathPlanner(GRID)
start_pos = (initial_pose[0], initial_pose[1])
goal_pos = (1.5, 1.5) # WHERE WE ARE HEADED
waypoints = planner.find_path(start_pos, goal_pos)

print("WAYPOINTS: ", waypoints)

# ------------------------------------------------------------------
# Main
# ------------------------------------------------------------------
def main():
    current_state = STATE[0] # Starts travelling to the station 
    print("Group-project controller started.")
    print("Mission:", MISSION)
    print("target:", target)
    print("Stations:", [s["id"] for s in CONFIG["stations"]])
    print("Camera:", camera.getWidth(), "x", camera.getHeight())
    # print(move_towards_point((1.0, 1.0), get_pose())) # <- only works inside the loop
    
    # Okay now this loop moves the robot towards the target
    
    station_index = 0
    waypoint_index = 0
    waypoints = []
    
    while robot.step(timestep) != -1:
        
        # Need this for everything really
        pose = get_pose()
        
        if current_state == "TRAVEL_TO_STATION":
            if station_index >= len(CONFIG["stations"]):
                set_speed(0.0, 0.0)
                # continue saves from breaking
                continue
            station = CONFIG["stations"][station_index]
            
            if not waypoints:  # haven't planned a path to this station yet
                waypoints = planner.find_path((pose[0], pose[1]), station["observe"])
                waypoint_index = 0

            if waypoint_index >= len(waypoints):
                set_speed(0.0, 0.0)
                # finished all waypoints for this station, move to the next one
                print(f"ROBOT REACHED STATION {station['id']}")
                
                current_state = "ORIENT_TOWARD_STATION"
                
                waypoints = []
                # So where this time.sleep is I imagine us making the camera facing towards the station 
                # occurring here, and then also the object recognisntion happens here
                continue
            
            current_target = waypoints[waypoint_index]
            
            if has_arrived(pose, current_target):
                print(f"ROBOT REACHED WAYPOINT {waypoint_index} of {len(waypoints)}")
                waypoint_index += 1
            
            else:
                left, right = move_towards_point(current_target, pose)
                set_speed(left, right)
                
        elif current_state == "ORIENT_TOWARD_STATION":
            # rotate in place until yaw ≈ station["observe_yaw"]
            # once aligned, switch to:
            current_state = "INSPECTING_STATION"
            pass
            
        elif current_state == "INSPECTING_STATION":
            station_index += 1
            # capture frame, call identify_target()
            # if match: current_state = "DONE"
            # if no match: station_index += 1, waypoints = [], current_state = "TRAVEL_TO_STATION"
            pass
            
        elif current_state == "DONE":
            set_speed(0.0, 0.0)
        

if __name__ == "__main__":
    main()
