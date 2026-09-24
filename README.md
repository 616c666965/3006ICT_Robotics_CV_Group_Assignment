# 3006ICT Robotics and Computer Vision

Group Project - Final Student Materials

**Team**

| Name | Student ID |
|------|-----------|
| Alfie McNamee | S5374969 |
| Ayush Lal | S5409751 |
| Ragib Ashab | S5404903 |

## Setup and Running

**Requirements**

- Webots R2025a
- Python 3.10 or later with `numpy` and `opencv-python`

```
pip install numpy opencv-python
```

No pretrained models or downloads are needed. The controller uses only the
files in this folder, with relative paths.

**Steps**

1. Run `python tools/check_python_environment.py` from the
   `3006ICT_group_project_webots` folder. It prints the Python path and checks
   that `numpy` and `cv2` import correctly.
2. In Webots, set **Preferences → General → Python command** to that Python
   path. Put the path in double quotes if it contains spaces.
3. Set the mission target in `config/assessment_mission.json`.
4. Open a world from `3006ICT_group_project_webots/worlds/` and press Play.

**Controller files** (`controllers/group_project_controller/`)

| File | Purpose |
|------|---------|
| `group_project_controller.py` | Main state machine: travel, orient, inspect, stop |
| `path_planner.py` | A* path planning on the occupancy grid |
| `vision.py` | ORB feature matching against the target reference images |
| `project_utils.py` | Config loading and grid/world coordinate helpers (provided) |

## Training Worlds

- `worlds/training_start_A.wbt`
- `worlds/training_start_B.wbt`
- `worlds/training_start_C.wbt`

## Environment

- 4 m x 4 m e-puck-scale arena
- 8 observation stations: 4 boundary + 4 interior
- 5 navigation barriers B1-B5
- every B1-B5 vertical face carries a non-target distractor image
- 40 x 40 occupancy grid at 0.10 m/cell
- e-puck camera, ps0-ps7, GPS and InertialUnit

## Observation Targets

- soda_can
- coffee_mug
- backpack
- fire_extinguisher
- camera
- running_shoe
- headphones
- wall_clock

See `targets/target_reference.png`.

The images on B1-B5 are non-target visual distractors. They are not valid
mission targets.

## Mission Input

The target identity is provided in:

```
config/assessment_mission.json
```

Example:

```json
{"target": "camera"}
```

Your controller should read the mission target from this configuration rather
than requiring the instructor to edit your source code.

## Assessment

The target-to-station assignment may change in assessment worlds. Do not
assume that a target is always at the same station.

Do not use Webots Camera Recognition or equivalent simulator ground-truth
object identity to identify the target.

Keep the supplied folder structure unchanged.
