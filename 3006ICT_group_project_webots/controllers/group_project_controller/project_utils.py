"""Coordinate helpers for the 3006ICT group-project world."""

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CONFIG = json.loads((ROOT / "config" / "project_config.json").read_text())

X_MIN = CONFIG["arena"]["x_min"]
Y_MAX = CONFIG["arena"]["y_max"]
RES = CONFIG["arena"]["resolution"]


def world_to_grid(x, y):
    """World (x, y) -> occupancy-grid (row, col)."""
    col = int((x - X_MIN) / RES)
    row = int((Y_MAX - y) / RES)
    return row, col


def grid_to_world(row, col):
    """Occupancy-grid cell -> world coordinate at the cell centre."""
    x = X_MIN + (col + 0.5) * RES
    y = Y_MAX - (row + 0.5) * RES
    return x, y


def station_by_id(station_id):
    return next(s for s in CONFIG["stations"] if s["id"] == station_id)


def start_by_id(start_id):
    return next(s for s in CONFIG["starts"] if s["id"] == start_id)
