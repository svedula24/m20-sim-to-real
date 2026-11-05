"""Coverage-tour planner over an occupancy grid (pure NumPy + SciPy, no ROS).

plan_tour() puts one stop per spacing x spacing tile of reachable, open floor (the most open cell of the tile,
at least `clearance` from any obstacle), orders the stops by walking distance (nearest-neighbour seed, then 2-opt),
closes the loop back at the start, and inserts via-points where the straight leg between two stops would thread a
gap narrower than the wide route (Nav2 takes the shortest path, which once cut a 0.5 m squeeze and grazed boxes).
Extracted from the simulator's geometry tools so the mapping tour runs the same on a real site map.
"""
import math

import numpy as np
from scipy import ndimage

def geodesic(free, start):
    """4-connected BFS distance (cells) over `free` from start; -1 = unreachable."""
    from collections import deque
    dist = np.full(free.shape, -1, np.int32)
    dist[start] = 0
    q = deque([start])
    R, C = free.shape
    while q:
        r, c = q.popleft()
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            rr, cc = r + dr, c + dc
            if 0 <= rr < R and 0 <= cc < C and free[rr, cc] and dist[rr, cc] < 0:
                dist[rr, cc] = dist[r, c] + 1
                q.append((rr, cc))
    return dist

def plan_tour(occ, floor, spawn, cell, clearance, spacing):
    """Exploration tour: one waypoint per spacing x spacing tile of reachable floor (the
    most open cell in the tile, >= clearance from any box), ordered greedily by WALKING
    distance from spawn, ending back at spawn. Returns [(r, c), ...] incl. final spawn."""
    clear = ndimage.distance_transform_edt(~occ) * cell
    free = floor & (clear >= clearance)
    free[spawn] = True
    reach = geodesic(free, spawn) >= 0
    tile = max(1, int(round(spacing / cell)))
    pts = []
    for r0 in range(0, occ.shape[0], tile):
        for c0 in range(0, occ.shape[1], tile):
            blk = np.where(reach[r0:r0 + tile, c0:c0 + tile], clear[r0:r0 + tile, c0:c0 + tile], -1)
            if blk.max() > 0:
                r, c = np.unravel_index(np.argmax(blk), blk.shape)
                pts.append((r0 + r, c0 + c))
    # tile-edge neighbours land within cm of each other: keep the most open of any pair
    # closer than spacing/2 (and never one on top of spawn)
    pts.sort(key=lambda p: -clear[p])
    kept, min_d = [], (spacing / 2) / cell
    for p in pts:
        if all(math.dist(p, k) >= min_d for k in kept + [spawn]):
            kept.append(p)
    # closed loop spawn -> stops -> spawn: walking-distance matrix, nearest-neighbour seed,
    # then 2-opt (reverse any segment that shortens the loop) until no move helps
    nodes = [spawn] + kept
    fields = [geodesic(free, p) for p in nodes]
    D = np.array([[f[q] if f[q] >= 0 else 10 ** 6 for q in nodes] for f in fields], float)
    route, left = [0], set(range(1, len(nodes)))
    while left:
        nxt = min(left, key=lambda j: D[route[-1], j])
        route.append(nxt); left.discard(nxt)
    route.append(0)
    improved = True
    while improved:
        improved = False
        for i in range(1, len(route) - 2):
            for j in range(i + 1, len(route) - 1):
                a_, b_, c_, d_ = route[i - 1], route[i], route[j], route[j + 1]
                if D[a_, c_] + D[b_, d_] < D[a_, b_] + D[c_, d_] - 1e-9:
                    route[i:j + 1] = route[i:j + 1][::-1]
                    improved = True
    # Via-points: Nav2 takes the SHORTEST path between consecutive stops, which can thread
    # a squeeze the wide (>= clearance) route avoids — measured 2026-10-05: the stop9->home
    # leg cut through a ~0.5 m gap and grazed boxes every lap. Where the straight segment
    # leaves the wide corridor, follow the wide geodesic and add string-pulled via-points.
    out = []                                   # (cell, is_via)
    for a_i, b_i in zip(route[:-1], route[1:]):
        a, b = nodes[a_i], nodes[b_i]
        if not line_of_sight(free, a, b):
            path = descend(fields[b_i], a)
            k = 0
            while k < len(path) - 1:          # string pulling: farthest visible path cell
                j = len(path) - 1
                while j > k + 1 and not line_of_sight(free, path[k], path[j]):
                    j -= 1
                if j < len(path) - 1:
                    out.append((path[j], True))
                k = j
        out.append((b, False))
    # drop via-points within 1 m of a neighbour: such short hops don't change Nav2's route,
    # they just add stop-and-turn jitter
    min_hop = 1.0 / cell
    keep = []
    for i, (p, via) in enumerate(out):
        prev = keep[-1][0] if keep else spawn
        nxt = out[i + 1][0] if i + 1 < len(out) else None
        if via and (math.dist(p, prev) < min_hop or (nxt is not None and math.dist(p, nxt) < min_hop)):
            continue
        keep.append((p, via))
    return [p for p, _ in keep]

def line_of_sight(free, a, b):
    """True if the straight segment a->b (grid cells) stays inside `free`."""
    n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1])) * 2) + 1
    rr = np.rint(np.linspace(a[0], b[0], n)).astype(int)
    cc = np.rint(np.linspace(a[1], b[1], n)).astype(int)
    return bool(free[rr, cc].all())

def descend(field, start):
    """Walk downhill on a BFS distance field from start to its source (0). Cell path."""
    path, cur = [start], start
    R, C = field.shape
    while field[cur] > 0:
        r, c = cur
        nbrs = [(r + dr, c + dc) for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1))
                if 0 <= r + dr < R and 0 <= c + dc < C and field[r + dr, c + dc] >= 0]
        cur = min(nbrs, key=lambda p: field[p])
        path.append(cur)
    return path
