"""Independent cost/geometry references and saved original-D* state invariants."""
import math

from usv_uuc_cooperation.metrics.x2_check import observed_cells, certify_graph, reference_distances
from usv_uuc_cooperation.planning.x2_search import ObservedGraph, astar_search


def value(x):
    return math.inf if x is None else x


def state(snapshot):
    return {(x, y): (value(h), value(p), tag, back)
            for x, y, h, p, tag, back in snapshot['h_p_tag_back']}


def check_original_dstar(config, rows, graph_records):
    graphs, references, previous, cache_ids = {}, {}, {}, {}
    reference_rows = []
    for record in graph_records:
        graph = ObservedGraph({'map': config['map']}, record['blocked'], record['version'])
        assert graph.sha256 == record['sha256']
        certify_graph(graph, tuple(config['tasks_m']['T03']))
        graphs[record['version']] = graph
    for index, row in enumerate(rows, 1):
        assert row['query_id'] == index and row['algorithm'] == 'Dstar'
        cells, version = observed_cells(config, row['time_s'])
        graph = graphs[version]
        assert row['graph_version'] == version and graph.blocked == cells
        assert row['graph_sha256'] == graph.sha256
        start, goal = tuple(row['start']), tuple(row['goal'])
        if row['status'] == 'INVALID_ENDPOINT':
            assert start not in graph.edges or goal not in graph.edges
            assert row['path'] is None and row['state_id'] is None
            continue
        before, seeded, after = (row[k] for k in ('before_state', 'after_edge_updates', 'after_state'))
        a, seed, final = map(state, (before, seeded, after))
        cache = row['owner'], goal
        prior = previous.get(row['state_id'])
        if prior is None:
            assert row['mode'] in ('INITIALIZE', 'GOAL_RESET') and cache not in cache_ids
            assert all(h == math.inf and tag == 'NEW' and back is None
                       for node, (h, p, tag, back) in a.items() if node != goal)
            assert a[goal][:3] == (0, 0, 'OPEN')
            cache_ids[cache] = row['state_id']
        else:
            assert prior == before and cache_ids[cache] == row['state_id']
            assert row['mode'] not in ('INITIALIZE', 'GOAL_RESET')
            assert row['state_initialization_ns'] == 0
        assert before['goal'] == after['goal'] == list(goal)
        assert after['start'] == list(start) and after['graph_sha256'] == graph.sha256
        old_graph = graphs[before['graph_version']]
        changes = []
        for node in a:
            old, new = dict(old_graph.edges.get(node, ())), dict(graph.edges.get(node, ()))
            for end in sorted(old.keys() | new.keys()):
                c, d = old.get(end, math.inf), new.get(end, math.inf)
                if c != d:
                    changes.append({'from': list(node), 'to': list(end),
                        'old_cost_m': None if math.isinf(c) else c,
                        'new_cost_m': None if math.isinf(d) else d})
        assert changes == row['changed_edges']
        heads = {tuple(r['to']) for r in changes}
        assert heads == set(map(tuple, row['changed_heads']))
        # Seeding changes queue membership, never clears previous h/backpointers.
        for node, (h, p, tag, back) in a.items():
            assert seed[node][0] == h and seed[node][3] == back
            if node in heads and tag == 'CLOSED':
                assert seed[node][1:3] == (h, 'OPEN')
            else:
                assert seed[node] == a[node]
        for snap, vals in ((before, a), (seeded, seed), (after, final)):
            queue = {(x, y): value(k) for x, y, k in snap['queue']}
            assert set(queue) == {node for node, v in vals.items() if v[2] == 'OPEN'}
            assert all(k == min(vals[node][0], vals[node][1]) for node, k in queue.items())
        assert final[goal][0] == 0 and final[goal][3] is None
        trace = row['expansion_trace']
        assert row['expanded_nodes'] == len(trace)
        assert row['heap_pops'] == len(trace) + row['stale_pops']
        assert row['unique_expanded_nodes'] == len({tuple(e['node']) for e in trace})
        assert row['queue_insertions'] == len(row['updates'])
        for event in trace:
            h, p = value(event['h_before_m']), value(event['p_before_m'])
            assert event['key_m'] == min(h, p)
            assert event['category'] == ('RAISE' if p < h - 1e-10 else 'LOWER')
        for event in row['updates']:
            if event['tag_before'] == 'CLOSED':
                assert event['p_after_m'] == event['h_before_m']
        cache_key = version, goal
        if cache_key not in references:
            references[cache_key] = reference_distances(graph, goal)
        expected = references[cache_key].get(start)
        baseline = astar_search(start, goal, graph)
        assert (expected is not None) == (row['path'] is not None) == (baseline['path'] is not None)
        if expected is not None:
            path = row['path']
            assert path[0] == list(start) and path[-1] == list(goal)
            assert len(set(map(tuple, path))) == len(path)
            assert final[start][2] == 'CLOSED'
            for x, y in zip(path, path[1:]):
                assert tuple(y) in dict(graph.edges[tuple(x)])
                assert final[tuple(x)][3] == y
            length = math.fsum(math.dist(x, y) for x, y in zip(path, path[1:]))
            for cost in (length, final[start][0], row['distance_m'], row['search_cost_m'], baseline['distance_m']):
                assert math.isclose(cost, expected, rel_tol=0, abs_tol=1e-9)
        reference_rows.append({'query_id': index, 'status': 'PASS', 'graph_version': version,
            'dstar_cost_m': row['distance_m'], 'astar_cost_m': baseline['distance_m'],
            'astar_effective_pops': baseline['expanded_nodes'], 'astar_search_ns': baseline['search_ns']})
        previous[row['state_id']] = after
    return {'status': 'PASS', 'query_count': len(rows), 'graph_versions': sorted(graphs),
        'state_continuity_goal_isolation_queue_keys': 'PASS', 'prior_h_preserved_before_insert': 'PASS',
        'changed_arcs_seed_without_clearing_state': 'PASS', 'safe_backpointer_chain_and_exact_cost': 'PASS',
        'independent_full_reference_and_repeated_astar': 'PASS', 'no_future_observation': 'PASS',
        'reference_role': 'offline validation only; never supplies production D* paths',
        'astar_queries': reference_rows}
