"""Independent observed-graph, shortest-cost and saved-execution checks."""
import heapq
import math

from usv_uuc_cooperation.env.obstacle import forbidden_nodes
from usv_uuc_cooperation.metrics.integrated_check import safe_segment
from usv_uuc_cooperation.metrics.x1_integrated_check import check_result as accepted_execution_check
from usv_uuc_cooperation.planning.grid_planner import heuristic
from usv_uuc_cooperation.planning.x2_search import ObservedGraph


def observed_cells(config, time_s):
    cells, version = forbidden_nodes(config), 0
    for event in sorted(config['events'], key=lambda e: (e['time_s'], e['event_id'])):
        if event['time_s'] <= time_s and event['type'] == 'MAP_UPDATE':
            cells |= set(map(tuple, event['add_cells']))
            cells -= set(map(tuple, event['remove_cells']))
            version += 1
    return cells, version


def certify_graph(graph, goal):
    """Independently reconstruct edge eligibility with slab geometry and check h."""
    c, blocked = graph.config, graph.blocked
    directions = [(dx, dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1) if dx or dy]
    m = c['map']
    nodes = {(x, y) for x in range(m['x_range'][0], m['x_range'][1] + 1)
             for y in range(m['y_range'][0], m['y_range'][1] + 1)
             if safe_segment((x, y), (x, y), c, blocked)}
    assert set(graph.edges) == nodes
    for a, edges in graph.edges.items():
        expected = {}
        for dx, dy in directions:
            b = a[0] + dx, a[1] + dy
            if b not in nodes or (dx and dy and ((a[0] + dx, a[1]) in blocked or (a[0], a[1] + dy) in blocked)):
                continue
            if safe_segment(a, b, c, blocked):
                expected[b] = math.dist(a, b)
        assert dict(edges) == expected
        for b, w in edges:
            assert w > 0 and math.isclose(w, dict(graph.edges[b])[a], abs_tol=1e-12)
            assert heuristic(a, goal, 8) <= w + heuristic(b, goal, 8) + 1e-12
    assert heuristic(goal, goal, 8) == 0
    return len(nodes)


def reference_distances(graph, goal):
    """Full single-source reference; no production search loop or early goal exit."""
    if goal not in graph.edges:
        return {}
    distance, queue = {goal: 0.0}, [(0.0, goal)]
    settled = set()
    while queue:
        cost, node = heapq.heappop(queue)
        if node in settled:
            continue
        settled.add(node)
        for end, weight in graph.edges[node]:
            candidate = cost + weight
            if candidate < distance.get(end, math.inf):
                distance[end] = candidate
                heapq.heappush(queue, (candidate, end))
    return distance


def check_searches(config, rows, graph_records):
    graphs, references = {}, {}
    for row in graph_records:
        graph = ObservedGraph(config, row['blocked'], row['version'])
        assert graph.sha256 == row['sha256']
        certify_graph(graph, tuple(config['tasks_m']['T03']))
        graphs[row['version']] = graph
    for index, row in enumerate(rows, 1):
        assert row['query_id'] == index
        cells, version = observed_cells(config, row['time_s'])
        assert version == row['graph_version'] and cells == graphs[version].blocked
        graph = graphs[version]
        assert row['graph_sha256'] == graph.sha256
        start, goal = tuple(row['start']), tuple(row['goal'])
        key = version, goal
        if key not in references:
            references[key] = reference_distances(graph, goal)
        cost = references[key].get(start)
        assert (cost is not None) == (row['path'] is not None)
        if row['path'] is not None:
            path = row['path']
            assert path[0] == list(start) and path[-1] == list(goal)
            assert all(tuple(b) in dict(graph.edges[tuple(a)]) for a, b in zip(path, path[1:]))
            actual = math.fsum(math.dist(a, b) for a, b in zip(path, path[1:]))
            assert math.isclose(actual, cost, abs_tol=1e-9)
            assert math.isclose(actual, row['distance_m'], abs_tol=1e-9)
            assert math.isclose(actual, row['search_cost_m'], abs_tol=1e-9)
            assert row['expansion_trace'][-1]['node'] == list(goal)
        assert row['expanded_nodes'] == len(row['expansion_trace'])
        assert row['unique_expanded_nodes'] == len({tuple(e['node']) for e in row['expansion_trace']})
        assert row['heap_pops'] == row['expanded_nodes'] + row['stale_pops']
        assert row['heap_pops'] <= row['heap_pushes'] and row['search_ns'] >= 0
        for e in row['expansion_trace']:
            h = heuristic(e['node'], goal, 8) if row['algorithm'] == 'Astar' else 0
            assert math.isclose(e['priority_m'], e['g_m'] + h, abs_tol=1e-9)
    return {'status': 'PASS', 'query_count': len(rows), 'graph_versions': sorted(graphs),
            'independent_complete_graph_geometry': 'PASS', 'all_query_shortest_cost_and_path': 'PASS',
            'octile_consistency_positive_edges_zero_at_goal': 'PASS', 'work_trace_counts': 'PASS',
            'timer_boundary': 'source-reviewed search-loop-only; no stable speed ranking inferred'}


def check_execution(result, searches, expected_initial_queues=None):
    base = accepted_execution_check(result)
    c, snaps = result['config'], result['snapshots']
    expected = c['x2']['initial_queues'] if expected_initial_queues is None else expected_initial_queues
    assert result['initial_queues'] == expected
    assert not ({'events', 'current_schedule', 'x2'} & set(result['planner_config_keys']))
    for snap in snaps:
        blocked, version = observed_cells(c, snap['time_s'])
        assert snap['graph_version'] == version
        assert set(map(tuple, snap['observed_blocked_cells'])) == blocked
        assert all(safe_segment(s['position_m'], s['position_m'], c, blocked) for s in snap['boats'].values())
    for row in result['steps']:
        blocked, _ = observed_cells(c, row['time_s'])
        assert safe_segment(row['start_m'], row['end_m'], c, blocked)
    by_id = {row['query_id']: row for row in searches}
    for d in result['replans']:
        blocked, version = observed_cells(c, d['time_s'])
        assert d['graph_version'] == version
        for leg in d['legs']:
            row = by_id[leg['search_query_id']]
            assert row['time_s'] == d['time_s'] and row['graph_version'] == version
            assert row['path'] == leg['grid_path'] and row['goal'] == leg['path'][-1]
            assert all(safe_segment(a, b, c, blocked) for a, b in zip(leg['path'], leg['path'][1:]))
    expected = sorted(c['events'], key=lambda e: (e['time_s'], 0 if e['type'] == 'UUV_FAILURE' else 1, e['event_id']))
    assert [(e['event_id'], e['time_s']) for e in result['event_log']] == [(e['event_id'], e['time_s']) for e in expected]
    return {**base, 'observed_map_versions_and_actual_segment_safety': 'PASS',
            'leg_to_search_query_traceability': 'PASS', 'initial_frozen_queues': 'PASS'}


def check_pair(groups):
    a, d = groups['Astar'], groups['Dijkstra']
    assert a['config'] == d['config'] and a['initial_queues'] == d['initial_queues']
    for key in ('time_s', 'graph_version', 'observed_blocked_cells', 'current_m_per_s', 'event_ids'):
        assert [s[key] for s in a['snapshots']] == [s[key] for s in d['snapshots']]
    return {'status': 'PASS', 'same_input_queues_observations_flow_strategy_and_window': 'PASS',
            'changed': 'grid search algorithm only; tied paths may affect actual position and later response'}
