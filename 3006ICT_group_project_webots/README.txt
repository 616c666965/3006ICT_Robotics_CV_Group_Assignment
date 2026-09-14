3006ICT Robotics and Computer Vision
Group Project - Final Student Materials
=======================================

TRAINING WORLDS
---------------
worlds/training_start_A.wbt
worlds/training_start_B.wbt
worlds/training_start_C.wbt

ENVIRONMENT
-----------
- 4 m x 4 m e-puck-scale arena
- 8 observation stations: 4 boundary + 4 interior
- 5 navigation barriers B1-B5
- every B1-B5 vertical face carries a non-target distractor image
- 40 x 40 occupancy grid at 0.10 m/cell
- e-puck camera, ps0-ps7, GPS and InertialUnit

OBSERVATION TARGETS
-------------------
soda_can
coffee_mug
backpack
fire_extinguisher
camera
running_shoe
headphones
wall_clock

See targets/target_reference.png.

The images on B1-B5 are non-target visual distractors. They are not valid
mission targets.

MISSION INPUT
-------------
The target identity is provided in:
    config/assessment_mission.json

Example:
    {"target": "camera"}

Your controller should read the mission target from this configuration rather
than requiring the instructor to edit your source code.

ASSESSMENT
----------
The target-to-station assignment may change in assessment worlds. Do not
assume that a target is always at the same station.

Do not use Webots Camera Recognition or equivalent simulator ground-truth
object identity to identify the target.

Keep the supplied folder structure unchanged.
