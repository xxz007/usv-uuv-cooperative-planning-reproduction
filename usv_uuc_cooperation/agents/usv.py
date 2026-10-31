"""Framework V01; functions extracted unchanged from accepted lesson sources.
课时资料/P3/P3-L2/源码/p3_l2_relay.py
"""
import math
from usv_uuc_cooperation.metrics.evaluator import require



def choose_target(strategy, usv, positions, connected, ages, distances, initial):
    if strategy == 'fixed':
        return list(initial), None, 'fixed_initial_position'
    if strategy == 'centroid':
        return [sum(p[k] for p in positions.values()) / len(positions) for k in (0, 1)], None, 'current_centroid'
    require(strategy == 'outage_priority', 'Unknown strategy')
    lost = [boat for boat in positions if not connected[boat]]
    if not lost:
        return list(usv), None, 'all_connected_hold'
    boat = min(lost, key=lambda b: (-ages[b], distances[b], b))
    return list(positions[boat]), boat, 'longest_age_then_distance_then_id'


def bounded_candidate(usv, target, limit):
    distance = math.dist(usv, target)
    if distance == 0:
        return list(usv)
    scale = min(distance, limit) / distance
    return [a + scale * (b - a) for a, b in zip(usv, target)]

