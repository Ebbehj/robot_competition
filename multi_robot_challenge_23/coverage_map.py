import cv2
import numpy as np

FACTOR = 10
ROBOT_CLEARANCE = 0.50
MAX_WALL_DISTANCE = 2.0
VIEWPOINT_STEP = 2

def find_wall_cells(msg, factor=FACTOR):
    info = msg.info
    grid = np.array(msg.data, dtype=np.int8).reshape(info.height, info.width)

    h = info.height // factor
    w = info.width // factor
    blocks = grid[:h * factor, :w * factor].reshape(h, factor, w, factor)

    occupied = (blocks >= 50).any(axis=(1, 3))
    free = (blocks == 0).all(axis=(1, 3))

    kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    near_free = cv2.dilate(free.astype(np.uint8), kernel) > 0
    wall_cells = occupied & near_free

    return wall_cells, free


def find_viewpoints(msg, free, starts, factor=FACTOR):
    info = msg.info
    cell_size = info.resolution * factor

    clearance = cv2.distanceTransform(free.astype(np.uint8), cv2.DIST_L2, 5) * cell_size
    safe = clearance >= ROBOT_CLEARANCE

    _, labels = cv2.connectedComponents(safe.astype(np.uint8))
    
    reachable = np.zeros_like(safe)

    for start_x, start_y in starts:
        start_col = int((start_x - info.origin.position.x) / cell_size)
        start_row = int((start_y - info.origin.position.y) / cell_size)
        label = labels[start_row, start_col]

        if label == 0:
            raise ValueError(f'Start ({start_x}, {start_y}) is not a safe cell')

        reachable |= labels == label

    candidates = reachable & (clearance <= MAX_WALL_DISTANCE)
    mask = np.zeros_like(candidates)
    mask[::VIEWPOINT_STEP, ::VIEWPOINT_STEP] = True
    rows, cols = np.nonzero(candidates & mask)

    xs = info.origin.position.x + (cols + 0.5) * cell_size
    ys = info.origin.position.y + (rows + 0.5) * cell_size

    return list(zip(xs, ys))