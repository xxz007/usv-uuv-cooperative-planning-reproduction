"""Saved-state and repeated-A* checks, independent of production D* repair."""
import math

from usv_uuc_cooperation.metrics.x2_check import observed_cells, certify_graph, reference_distances
from usv_uuc_cooperation.planning.grid_planner import heuristic
from usv_uuc_cooperation.planning.x2_search import ObservedGraph, astar_search


def number(value):
    return math.inf if value is None else value


def state_values(snapshot):
    return {(x, y): (number(g), number(rhs)) for x, y, g, rhs in snapshot['g_rhs']}


def check_dstar(config, rows, graph_records):
    graphs, references, previous, astar_rows = {}, {}, {}, []
    for r in graph_records:
        graph = ObservedGraph(config, r['blocked'], r['version'])
        assert graph.sha256 == r['sha256']
        certify_graph(graph, tuple(config['tasks_m']['T03']))
        graphs[r['version']] = graph
    for index, row in enumerate(rows, 1):
        assert row['query_id'] == index and row['algorithm'] == 'DstarLite'
        cells, version = observed_cells(config, row['time_s'])
        assert row['graph_version'] == version and cells == graphs[version].blocked
        graph = graphs[version]
        assert row['graph_sha256'] == graph.sha256
        start, goal = tuple(row['start']), tuple(row['goal'])
        if row['status'] == 'INVALID_ENDPOINT':
            assert start not in graph.edges or goal not in graph.edges
            assert row['path'] is None and row['state_id'] is None
            continue
        before, seeded, after = (row[k] for k in ('before_state', 'after_edge_updates', 'after_state'))
        prior = previous.get(row['state_id'])
        if prior is not None:
            assert row['mode'] not in ('INITIALIZE', 'GOAL_RESET')
            assert before == prior
            assert row['state_initialization_ns'] == 0
        else:
            assert row['mode'] in ('INITIALIZE', 'GOAL_RESET')
            assert all(g is None for _, _, g, _ in before['g_rhs'])
            assert all(rhs == (0 if (x, y) == goal else None) for x, y, _, rhs in before['g_rhs'])
            assert before['km_m'] == 0
        assert tuple(before['goal']) == tuple(after['goal']) == goal
        assert tuple(after['start']) == start and after['graph_version'] == version
        assert after['graph_sha256'] == graph.sha256
        expected_km = before['km_m'] + heuristic(before['start'], start, 8)
        assert math.isclose(after['km_m'], expected_km, abs_tol=1e-9)
        old_graph = graphs[before['graph_version']]
        old, seed, final = map(state_values, (before, seeded, after))
        expected_edges, tails = [], set()
        for a in old:
            prior_edges, current_edges = dict(old_graph.edges.get(a, ())), dict(graph.edges.get(a, ()))
            for b in sorted(prior_edges.keys() | current_edges.keys()):
                x, y = prior_edges.get(b, math.inf), current_edges.get(b, math.inf)
                if x != y:
                    tails.add(a)
                    expected_edges.append({'from': list(a), 'to': list(b),
                                           'old_cost_m': x if math.isfinite(x) else None,
                                           'new_cost_m': y if math.isfinite(y) else None})
        assert expected_edges == row['changed_edges']
        assert tails == set(map(tuple, row['changed_tails']))
        for a, (g, rhs) in seed.items():
            assert g == old[a][0], 'Edge updates must preserve g until repair'
            expected_rhs = 0 if a == goal else min((w + old[b][0] for b, w in graph.edges.get(a, ())), default=math.inf)
            assert rhs == (expected_rhs if a in tails else old[a][1])
        assert final[start][0] == final[start][1]
        assert final[goal][1] == 0
        queue_nodes = {(x, y) for x, y, _, _ in after['queue']}
        assert queue_nodes == {p for p, (g, rhs) in final.items() if g != rhs}
        trace = row['expansion_trace']
        assert row['expanded_nodes'] == len(trace)
        assert row['unique_expanded_nodes'] == len({tuple(e['node']) for e in trace})
        assert row['heap_pops'] == len(trace) + row['stale_pops']
        assert row['update_vertex_calls'] == len(row['rhs_updates'])
        for e in trace:
            g, rhs = number(e['g_before_m']), number(e['rhs_before_m'])
            if e['action'] == 'SET_G_TO_RHS':
                assert g > rhs and e['g_after_m'] == e['rhs_before_m']
            elif e['action'] == 'INVALIDATE_G':
                assert g <= rhs and e['g_after_m'] is None
            else:
                assert e['action'] == 'REKEY'
                assert tuple(map(number, e['old_key'])) < tuple(map(number, e['new_key']))
                assert e['g_after_m'] == e['g_before_m']
        key = version, goal
        if key not in references:
            references[key] = reference_distances(graph, goal)
        cost = references[key].get(start)
        baseline = astar_search(start, goal, graph)
        assert (cost is not None) == (row['path'] is not None) == (baseline['path'] is not None)
        if cost is not None:
            path = row['path']
            assert path[0] == list(start) and path[-1] == list(goal)
            assert all(tuple(b) in dict(graph.edges[tuple(a)]) for a, b in zip(path, path[1:]))
            for actual in (row['distance_m'], row['search_cost_m'], baseline['distance_m']):
                assert math.isclose(actual, cost, abs_tol=1e-9)
        astar_rows.append({'query_id': index, 'graph_version': version, 'graph_sha256': graph.sha256,
                           'start': list(start), 'goal': list(goal), 'dstar_cost_m': row['distance_m'],
                           'astar_cost_m': baseline['distance_m'], 'astar_search_ns': baseline['search_ns'],
                           'astar_effective_pops': baseline['expanded_nodes'], 'status': 'PASS'})
        previous[row['state_id']] = after
    return {'status': 'PASS', 'query_count': len(rows), 'graph_versions': sorted(graphs),
            'independent_geometry_and_full_reference': 'PASS', 'repeated_same_graph_astar': 'PASS',
            'g_rhs_state_continuity_and_goal_reset': 'PASS', 'moving_start_km': 'PASS',
            'changed_edge_tails_seeded_without_clearing_g': 'PASS',
            'inconsistent_queue_and_trace_counts': 'PASS', 'no_future_observation': 'PASS',
            'reference_role': 'offline validation only, never used to supply D* Lite paths',
            'astar_queries': astar_rows}
