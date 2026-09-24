import cv2
import numpy as np
from pathlib import Path
from project_utils import ROOT

orb = cv2.ORB_create(
    nfeatures=600,
    scaleFactor=1.2,
    nlevels=12,
    edgeThreshold=15,
    patchSize=15,
    fastThreshold=7
)
brute_force = cv2.BFMatcher(cv2.NORM_HAMMING)

TARGET_NAMES = [
    "soda_can", "coffee_mug", "backpack", "fire_extinguisher",
    "camera", "running_shoe", "headphones", "wall_clock"
]

reference_descriptors = {}
for name in TARGET_NAMES:
    path = ROOT / "textures" / f"target_{name}.png"
    reference_image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    keypoints, descriptors = orb.detectAndCompute(reference_image, None)
    reference_descriptors[name] = {
        "keypoints": keypoints,
        "descriptors": descriptors,
    }

def identify_target(frame_bgr, minimum_good_matches=4):
    frame_bgr = cv2.resize(frame_bgr, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
    grayscale_version = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    keypoints, descriptors = orb.detectAndCompute(grayscale_version, None)
    if descriptors is None or len(descriptors) < minimum_good_matches:
        return None

    best_name = None
    best_score = (0, 0)  # (inliers, good_matches)

    for name, ref_data in reference_descriptors.items():
        if ref_data["descriptors"] is None:
            continue

        # Lowe's ratio test (replaces the loose distance < 60 threshold)
        matches = brute_force.knnMatch(descriptors, ref_data["descriptors"], k=2)
        good_matches = [
            m[0] for m in matches 
            if len(m) == 2 and m[0].distance < 0.78 * m[1].distance
        ]

        if len(good_matches) < minimum_good_matches:
            continue

        # Spatial consistency check
        frame_pts = np.float32([keypoints[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        ref_pts = np.float32([ref_data["keypoints"][m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

        _, inliers = cv2.estimateAffinePartial2D(
            ref_pts, frame_pts, method=cv2.RANSAC, ransacReprojThreshold=4.0
        )
        inlier_count = int(np.sum(inliers)) if inliers is not None else 0

        consensus_ratio = inlier_count / len(good_matches)
        # Require at least 4 inliers and 35% consensus among candidate matches
        if inlier_count < minimum_good_matches or (inlier_count / len(good_matches)) < 0.35:
            continue

        # Rank by inliers first; break ties using total good matches
        score = (inlier_count, consensus_ratio)
        if score > best_score:
            best_score = score
            best_name = name

    if best_score[0] >= minimum_good_matches:
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