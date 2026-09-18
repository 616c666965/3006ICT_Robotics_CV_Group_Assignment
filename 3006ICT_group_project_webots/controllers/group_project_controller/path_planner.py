import heapq
import math
import cv2
import numpy as np

from project_utils import world_to_grid, grid_to_world


class PathPlanner:
    def __init__(self, raw_grid, inflation_radius=0):
        self.raw_grid = raw_grid
        self.rows, self.cols = raw_grid.shape
        
        if inflation_radius > 0:
            kernel = cv2.getStructuringElement(
                cv2.MORPH_CROSS, 
                (2 * inflation_radius + 1, 2 * inflation_radius + 1)
            )
            self.grid = cv2.dilate(raw_grid.astype(np.uint8), kernel, iterations=1)
        else:
            self.grid = raw_grid.copy().astype(np.uint8)

    def heuristic(self, a, b):
        """Euclidean distance between two grid cells."""
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def is_valid(self, r, c):
        """Check if cell is within arena limits and not an obstacle."""
        return 0 <= r < self.rows and 0 <= c < self.cols and self.grid[r, c] == 0

    def find_path(self, start_pos, goal_pos):
        """Find an A* path from start to goal in world coordinates."""
        start_cell = world_to_grid(start_pos[0], start_pos[1])
        goal_cell = world_to_grid(goal_pos[0], goal_pos[1])

        # If goal is inside an inflated cell, unmask the exact goal cell
        if 0 <= goal_cell[0] < self.rows and 0 <= goal_cell[1] < self.cols:
            self.grid[goal_cell[0], goal_cell[1]] = 0

        # Movement deltas and costs
        neighbors = [
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)
        ]

        # Priority queue storage
        open_set = [(self.heuristic(start_cell, goal_cell), start_cell)]
        came_from = {}
        g_score = {start_cell: 0.0}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal_cell:
                return self._reconstruct_path(came_from, current, goal_pos)

            r, c = current
            for dr, dc, move_cost in neighbors:
                nr, nc = r + dr, c + dc
                nbr = (nr, nc)

                # Skip invalid or blocked cells
                if not self.is_valid(nr, nc):
                    continue

                # Prevent cutting diagonal corners through obstacles
                if dr != 0 and dc != 0 and (self.grid[r, nc] == 1 or self.grid[nr, c] == 1):
                    continue

                tentative_g = g_score[current] + move_cost
                if tentative_g < g_score.get(nbr, float("inf")):
                    came_from[nbr] = current
                    g_score[nbr] = tentative_g
                    f_score = tentative_g + self.heuristic(nbr, goal_cell)
                    heapq.heappush(open_set, (f_score, nbr))

        return []

    def _reconstruct_path(self, came_from, current, exact_goal_pos):
        """Reconstruct the path from start to goal and convert to world coordinates."""
        grid_path = [current]
        while current in came_from:
            current = came_from[current]
            grid_path.append(current)
        grid_path.reverse()

        # Convert cells back to world coordinates
        world_waypoints = [grid_to_world(r, c) for r, c in grid_path]
        
        # Snap the final waypoint to the exact target coordinate
        world_waypoints[-1] = (exact_goal_pos[0], exact_goal_pos[1])
        return world_waypoints

if __name__ == "__main__":
    from project_utils import CONFIG, ROOT

    raw_grid = np.load(ROOT / "maps" / "occupancy_grid.npy")
    planner = PathPlanner(raw_grid, inflation_radius=0)

    start_pos = (-1.45, 0.0)
    for station in CONFIG["stations"]:
        path = planner.find_path(start_pos, station["observe"])
        print(f"{station['id']}: {len(path)} waypoints")