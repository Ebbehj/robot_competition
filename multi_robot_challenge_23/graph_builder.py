"""Koble sammen eksisterende observasjonspunkter i fire retninger.

Bruk samme factor og viewpoint_step som i coverage_map.py.
Returnerer graph til Dijkstra og positions til rutefølgeren.
"""

import math
import numpy as np

# Bruk samme innstillinger som lager observasjonspunktene.
from multi_robot_challenge_23.coverage_map import (
    FACTOR,
    VIEWPOINT_STEP,
    ROBOT_CLEARANCE,
)


def build_graph(
    map_msg,
    viewpoints,
    factor=FACTOR,
    viewpoint_step=VIEWPOINT_STEP,
    clearance=ROBOT_CLEARANCE,
):
    # Hent kartinformasjonen som coverage_map.py også bruker.
    resolution = map_msg.info.resolution
    width = map_msg.info.width
    height = map_msg.info.height
    origin_x = map_msg.info.origin.position.x
    origin_y = map_msg.info.origin.position.y

    if resolution <= 0 or factor < 1 or viewpoint_step < 1 or clearance < 0:
        raise ValueError("Ugyldig oppløsning, punktavstand eller klaring.")

    # Som coverage_map.py forutsetter vi at kartet ikke er rotert.
    rotation = map_msg.info.origin.orientation
    if any(abs(value) > 1e-8 for value in (rotation.x, rotation.y, rotation.z)):
        raise ValueError("Roterte kart støttes ikke av denne versjonen.")

    # Gjør kartets lange liste om til rader og kolonner.
    grid = np.array(map_msg.data).reshape(height, width)
    cell_size = resolution * factor

    # STEG 1: Gi hvert eksisterende punkt en ID.
    graph = {}       # Node-ID -> naboer og avstander.
    positions = {}   # Node-ID -> (x, y) i kartet.
    node_at = {}     # (rad, kolonne) -> node-ID.

    for x, y in viewpoints:
        # Punktene ligger midt i grove celler, derfor -0.5.
        col = round((x - origin_x) / cell_size - 0.5)
        row = round((y - origin_y) / cell_size - 0.5)

        # Oppdag innstillinger som ikke passer med punktlisten.
        expected_x = origin_x + (col + 0.5) * cell_size
        expected_y = origin_y + (row + 0.5) * cell_size
        if abs(x - expected_x) > 1e-6 or abs(y - expected_y) > 1e-6:
            raise ValueError("factor passer ikke med observasjonspunktene.")
        if row % viewpoint_step != 0 or col % viewpoint_step != 0:
            raise ValueError("viewpoint_step passer ikke med punktene.")
        if (row, col) in node_at:
            raise ValueError("To punkter ligger i samme celle.")

        # Cellebaserte ID-er endres ikke hvis punktlisten sorteres annerledes.
        node_id = f"node_r{row}_c{col}"
        node_at[(row, col)] = node_id
        positions[node_id] = (float(x), float(y))
        graph[node_id] = {}

    # STEG 2: Finn naboer én punktavstand unna.
    for (row, col), node_id in node_at.items():
        neighbor_cells = [
            (row, col + viewpoint_step),  # Høyre.
            (row + viewpoint_step, col),  # Positiv y-retning.
        ]

        for neighbor_cell in neighbor_cells:
            if neighbor_cell not in node_at:
                continue

            neighbor_id = node_at[neighbor_cell]
            start = positions[node_id]
            end = positions[neighbor_id]

            # STEG 3: Kontroller fri vei, inkludert klaring på sidene.
            if not corridor_is_free(
                grid, resolution, origin_x, origin_y, start, end, clearance
            ):
                continue

            # STEG 4: Lagre forbindelsen med avstand som kostnad.
            distance = math.hypot(end[0] - start[0], end[1] - start[1])
            graph[node_id][neighbor_id] = distance
            graph[neighbor_id][node_id] = distance

            # Begge retninger er lagt inn. Dermed får vi også venstre/ned,
            # selv om vi bare leter mot høyre og positiv y over.

    return graph, positions


def corridor_is_free(grid, resolution, origin_x, origin_y, start, end, clearance):
    """Kontroller et rektangel rundt den rette forbindelsen.

    Vi bruker originalkartet, slik at vegger mellom punktene ikke overses.
    Hele rektangelet må være ledig. Dette er en konservativ kontroll:
    den kan avvise forbindelser som roboten i praksis kunne passert.
    """
    start_x, start_y = start
    end_x, end_y = end

    # Rektangelets grenser, relativt til kartets start.
    left = min(start_x, end_x) - origin_x - clearance
    right = max(start_x, end_x) - origin_x + clearance
    bottom = min(start_y, end_y) - origin_y - clearance
    top = max(start_y, end_y) - origin_y + clearance

    # Hele området må være innenfor kartet.
    height, width = grid.shape
    if (
        left <= 0 or bottom <= 0
        or right >= width * resolution or top >= height * resolution
    ):
        return False

    # Finn cellene rektangelet berører, inkludert ved cellegrenser.
    first_col = max(0, math.ceil(left / resolution) - 1)
    last_col = min(width - 1, math.floor(right / resolution))
    first_row = max(0, math.ceil(bottom / resolution) - 1)
    last_row = min(height - 1, math.floor(top / resolution))

    # Hent bare kartutsnittet rundt forbindelsen.
    # +1 fordi siste indeks ellers ikke inkluderes i Python-utklippet.
    corridor = grid[first_row:last_row + 1, first_col:last_col + 1]

    # 0 betyr ledig. Ukjente celler og hindringer gir False.
    return bool(np.all(corridor == 0))
