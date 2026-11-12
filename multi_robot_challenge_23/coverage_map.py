import cv2
import numpy as np
from collections import deque


FACTOR = 10
ROBOT_CLEARANCE = 0.50
MAX_WALL_DISTANCE = 2.0
VIEWPOINT_STEP = 2

VIEW_DISTANCE = 3.0

SPAWN_EXCLUDE_RADIUS = 2.0

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

def wall_step_distance(wall_cells, free, max_steps):
    h, w = wall_cells.shape
    dist = np.full((h, w), -1, dtype=int)
    q = deque()

    for r, c in zip(*np.nonzero(wall_cells)):
        dist[r, c] = 0
        q.append((r, c))

    while q:
        r, c = q.popleft()
        if dist[r, c] >= max_steps:
            continue
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nr, nc = r + dr, c + dc
            if 0 <= nr < h and 0 <= nc < w and free[nr, nc] and dist[nr, nc] == -1:
                dist[nr, nc] = dist[r, c] + 1
                q.append((nr, nc))

    return dist


def find_reachable(msg, free, starts, factor=FACTOR):
    info = msg.info
    cell_size = info.resolution * factor

    clearance = cv2.distanceTransform(free.astype(np.uint8), cv2.DIST_L2, 5) * cell_size
    safe = clearance >= ROBOT_CLEARANCE

    _, labels = cv2.connectedComponents(safe.astype(np.uint8))
    reachable = np.zeros_like(safe)

    for start_x, start_y in starts:
        col = int((start_x - info.origin.position.x) / cell_size)
        row = int((start_y - info.origin.position.y) / cell_size)
        label = labels[row, col]

        if label == 0:
            raise ValueError(f'Start ({start_x}, {start_y}) is not a safe cell')

        reachable |= labels == label

    return reachable

def find_intrestpoints(msg):
    wall_cells, free = find_wall_cells(msg)
    cell_size = msg.info.resolution * FACTOR
    max_steps = int(VIEW_DISTANCE / cell_size)
    return wall_step_distance(wall_cells, free, max_steps)

def find_peaks(dist, radius_cells):
    """Greedy: keep the highest cell, suppress everything within radius, repeat."""
    h, w = dist.shape
    R = max(int(radius_cells), 0)

    rows, cols = np.nonzero(dist > 0)
    order = np.argsort(-dist[rows, cols], kind='stable')

    yy, xx = np.ogrid[-R:R + 1, -R:R + 1]
    disk = xx**2 + yy**2 <= R**2
    suppressed = np.zeros((h, w), dtype=bool)
    peaks = []

    for i in order:
        r, c = rows[i], cols[i]
        if suppressed[r, c]:
            continue
        peaks.append((r, c))

        r0, r1 = max(r - R, 0), min(r + R + 1, h)
        c0, c1 = max(c - R, 0), min(c + R + 1, w)
        suppressed[r0:r1, c0:c1] |= disk[r0 - (r - R):r1 - (r - R),
                                         c0 - (c - R):c1 - (c - R)]
    return peaks

def remove_near_spawn(msg, peaks, factor=FACTOR):
    info = msg.info
    cell_size = info.resolution * factor
    kept = []
    for r, c in peaks:
        x = info.origin.position.x + (c + 0.5) * cell_size
        y = info.origin.position.y + (r + 0.5) * cell_size
        if np.hypot(x, y) > SPAWN_EXCLUDE_RADIUS:
            kept.append((r, c))
    return kept