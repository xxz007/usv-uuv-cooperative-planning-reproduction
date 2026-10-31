"""Framework V01; functions extracted unchanged from accepted lesson sources.
课时资料/P3/P3-L4/源码/p3_l4_map.py
"""
from usv_uuc_cooperation.planning.astar import segment_touches_square



def forbidden_nodes(config):
    """Forbid each closed grid square intersecting any buffered circle."""
    grid = config['map']
    half = grid['spacing_m'] / 2
    blocked = set()
    for x in range(grid['x_range'][0], grid['x_range'][1] + 1):
        for y in range(grid['y_range'][0], grid['y_range'][1] + 1):
            for circle in grid['circles']:
                cx, cy = circle['center_m']
                dx = max(abs(x - cx) - half, 0)
                dy = max(abs(y - cy) - half, 0)
                if dx * dx + dy * dy <= (circle['radius_m'] + grid['safety_buffer_m']) ** 2:
                    blocked.add((x, y))
                    break
    return blocked


def unsafe_reason(start, end, config, blocked):
    grid = config['map']
    if any(not (grid['x_range'][0] <= p[0] <= grid['x_range'][1]
                and grid['y_range'][0] <= p[1] <= grid['y_range'][1]) for p in (start, end)):
        return 'out_of_bounds'
    if any(segment_touches_square(start, end, cell, grid['spacing_m'] / 2)
           for cell in sorted(blocked)):
        return 'touches_forbidden_grid_square'
    return None

