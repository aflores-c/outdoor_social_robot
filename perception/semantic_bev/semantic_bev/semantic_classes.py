"""
Semantic class definitions for the BEV map.
Class IDs, display colors, OccupancyGrid costs, and fusion priority.
"""
from enum import IntEnum
import numpy as np


class SemanticClass(IntEnum):
    UNKNOWN    = 0
    FREE       = 1
    OBSTACLE   = 2
    SIDEWALK   = 3
    ROAD       = 4
    CROSSWALK  = 5
    GRASS      = 6
    PEDESTRIAN = 7
    VEHICLE    = 8
    BICYCLE    = 9
    CURB       = 10


NUM_CLASSES = len(SemanticClass)

CLASS_NAMES = {int(c): c.name.lower() for c in SemanticClass}

# BGR colors used by the debug image (OpenCV convention)
CLASS_COLORS_BGR: dict[int, tuple] = {
    int(SemanticClass.UNKNOWN):    (50,  50,  50),
    int(SemanticClass.FREE):       (210, 210, 210),
    int(SemanticClass.OBSTACLE):   (30,  30,  220),
    int(SemanticClass.SIDEWALK):   (160, 220, 244),
    int(SemanticClass.ROAD):       (110, 110, 110),
    int(SemanticClass.CROSSWALK):  (255, 255, 255),
    int(SemanticClass.GRASS):      (20,  160,  30),
    int(SemanticClass.PEDESTRIAN): (0,    0,  255),
    int(SemanticClass.VEHICLE):    (0,  200,   0),
    int(SemanticClass.BICYCLE):    (0,  165,  255),
    int(SemanticClass.CURB):       (0,  230,  230),
}

# nav_msgs/OccupancyGrid values: -1=unknown, 0=free, 1–100=occupied
CLASS_COSTS: dict[int, int] = {
    int(SemanticClass.UNKNOWN):    -1,
    int(SemanticClass.FREE):        0,
    int(SemanticClass.OBSTACLE):   90,
    int(SemanticClass.SIDEWALK):   20,
    int(SemanticClass.ROAD):       10,
    int(SemanticClass.CROSSWALK):  30,
    int(SemanticClass.GRASS):      65,
    int(SemanticClass.PEDESTRIAN): 100,
    int(SemanticClass.VEHICLE):    100,
    int(SemanticClass.BICYCLE):    95,
    int(SemanticClass.CURB):       70,
}

# Fusion priority: lower number wins (overrides) during cell merge
CLASS_PRIORITY: dict[int, int] = {
    int(SemanticClass.PEDESTRIAN): 1,
    int(SemanticClass.VEHICLE):    2,
    int(SemanticClass.BICYCLE):    3,
    int(SemanticClass.OBSTACLE):   4,
    int(SemanticClass.CURB):       5,
    int(SemanticClass.GRASS):      6,
    int(SemanticClass.CROSSWALK):  7,
    int(SemanticClass.SIDEWALK):   8,
    int(SemanticClass.ROAD):       9,
    int(SemanticClass.FREE):       10,
    int(SemanticClass.UNKNOWN):    99,
}

# Lookup table: array[class_id] → priority value (for fast vectorised ops)
_PRIORITY_LUT = np.array(
    [CLASS_PRIORITY.get(i, 99) for i in range(NUM_CLASSES)], dtype=np.int8
)

# Lookup table: array[class_id] → OccupancyGrid cost
_COST_LUT = np.array(
    [CLASS_COSTS.get(i, -1) for i in range(NUM_CLASSES)], dtype=np.int8
)


def higher_priority_scalar(a: int, b: int) -> int:
    """Return the class with lower priority-number (higher semantic importance)."""
    return a if CLASS_PRIORITY.get(a, 99) <= CLASS_PRIORITY.get(b, 99) else b


def merge_grids(base: np.ndarray, overlay: np.ndarray) -> np.ndarray:
    """
    Vectorised merge: for each cell keep whichever class has lower priority number.
    Both arrays must have dtype int8 and the same shape.
    """
    clamp_b = np.clip(base,    0, NUM_CLASSES - 1)
    clamp_o = np.clip(overlay, 0, NUM_CLASSES - 1)
    base_pri    = _PRIORITY_LUT[clamp_b]
    overlay_pri = _PRIORITY_LUT[clamp_o]
    return np.where(overlay_pri < base_pri, overlay, base).astype(np.int8)


def class_grid_to_cost(grid: np.ndarray) -> np.ndarray:
    """Convert class-ID grid (int8) to OccupancyGrid cost grid (int8)."""
    return _COST_LUT[np.clip(grid, 0, NUM_CLASSES - 1)]
