import cv2
import numpy as np
from pathlib import Path
from project_utils import ROOT

orb = cv2.ORB_create(nfeatures=2000, scaleFactor=1.1, nlevels=12)!
brute_force = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

TARGET_NAMES = ["soda_can", "coffee_mug", "backpack", "fire_extinguisher",
                "camera", "running_shoe", "headphones", "wall_clock"]

reference_descriptors = {}
for name in TARGET_NAMES:
    path = ROOT / "textures" / f"target_{name}.png"
    reference_image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    keypoints, descriptors = orb.detectAndCompute(reference_image, None)
    reference_descriptors[name] = descriptors

def identify_target(frame_bgr, minimum_good_matches=15):
    grayscale_version = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    keypoints, descriptors = orb.detectAndCompute(grayscale_version, None)
    if descriptors is None:
        return None

    best_name = None
    best_count = 0

    for name, reference_descriptor in reference_descriptors.items():
        if reference_descriptor is None:
            continue
        matches = brute_force.match(descriptors, reference_descriptor)
        good_matches = [i for i in matches if i.distance < 60]
        min_distance = min((m.distance for m in matches), default=None)
        if len(good_matches) > best_count:
            best_count = len(good_matches)
            best_name = name

    if best_count >= minimum_good_matches:
        return best_name
    return None
    
    
# This is purely for my testing
if __name__ == "__main__":
    for name in TARGET_NAMES:
        test_path = ROOT / "textures" / f"target_{name}.png"
        test_img = cv2.imread(str(test_path))
        result = identify_target(test_img)
        print(f"Testing {name}.png -> identified as: {result}")

    print()
    distractor_dir = ROOT / "textures" / "distractors"
    for path in sorted(distractor_dir.glob("*.png")):
        test_img = cv2.imread(str(path))
        result = identify_target(test_img)
        print(f"Testing {path.name} -> identified as: {result}")