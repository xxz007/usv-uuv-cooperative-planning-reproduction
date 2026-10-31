"""Released task ordering plus the unchanged A* connector/greedy dispatcher."""
import copy
import math

from usv_uuc_cooperation.planning.continuous_routes import rebuild
from usv_uuc_cooperation.planning.x2_search import SearchSession


def scan_order(queue, tasks, config, time_s, protected=None):
    if len(queue) != len(set(queue)):
        raise ValueError('Duplicate task in scan queue')
    pending = [t for t in queue if tasks[t]['status'] != 'completed']
    if any(tasks[t]['release_s'] > time_s for t in pending):
        raise ValueError('Scan order must not receive unreleased tasks')
    m = config['map']
    def key(task):
        x, y = tasks[task]['position_m']
        row = (y - m['y_range'][0]) / m['spacing_m']
        if not math.isfinite(row) or not math.isclose(row, round(row), abs_tol=1e-9):
            raise ValueError('Scan task must lie on a grid row')
        row = int(round(row))
        return row, x if row % 2 == 0 else -x, task
    if protected is not None and protected not in pending:
        raise ValueError('Protected service head is absent from pending queue')
    return ([protected] if protected is not None else []) + sorted(
        [t for t in pending if t != protected], key=key)


class ScanAStarSession(SearchSession):
    """All grid searches remain Astar; fleet group is separately GridScanAstar."""
    def __init__(self, config):
        self.owner, self.task, self.query_role = None, None, None
        self.ordering_phase = 'INITIAL'
        super().__init__({'map': config['map']}, 'Astar')

    def observe(self, config, blocked, version):
        if {'events', 'current_schedule', 'x2'} & set(config):
            raise ValueError('Scan connector receives current physical data only')
        super().observe(config, blocked, version)

    def route_context(self, owner, task, query_role):
        self.owner, self.task, self.query_role = owner, task, query_role

    def __call__(self, start, goal, config, blocked):
        if {'events', 'current_schedule', 'x2'} & set(config):
            raise ValueError('Scan connector must not receive future tables')
        result = super().__call__(start, goal, config, blocked)
        self.rows[-1].update(owner=self.owner, task_id=self.task, query_role=self.query_role,
                            ordering_phase=self.ordering_phase)
        return result


def scan_rebuild(config, blocked, states, tasks, accepted, queues, time_s, reason, assign_new, planner):
    if {'events', 'current_schedule', 'x2'} & set(config):
        raise ValueError('Scan rebuild receives current observed state only')
    original = copy.deepcopy(queues)
    owners = {t: item['boat_id'] for t, item in tasks.items()}
    protected, retained = {}, {}
    for boat in config['uuc_ids']:
        if states[boat]['failure']:
            protected[boat], retained[boat] = None, []
            continue
        live = [t for t in queues[boat] if tasks[t]['status'] != 'completed']
        heads = [t for t in live if tasks[t]['status'] in ('arrived', 'servicing')
                 or tasks[t]['service_progress_s'] > 0]
        if len(heads) > 1 or (heads and heads[0] != live[0]):
            raise ValueError('Invalid active service head in scan input')
        protected[boat] = heads[0] if heads else None
        retained[boat] = scan_order(live, tasks, config, time_s, protected[boat])
    planner.ordering_phase = 'RETAINED_SCAN_AND_SHARED_DISPATCH'
    proposed, plans, dispatch = rebuild(config, blocked, states, tasks, accepted,
                                       retained, time_s, reason, assign_new, planner)
    ordered = {boat: scan_order(proposed[boat], tasks, config, time_s, protected[boat])
               for boat in config['uuc_ids']}
    if ordered != proposed:
        planner.ordering_phase = 'POST_DISPATCH_EXECUTABLE_SCAN'
        output, plans, decision = rebuild(config, blocked, states, tasks, accepted,
                                          ordered, time_s, reason, False, planner)
        # Preserve original ownership transitions, not the already reassigned owner.
        for row in decision['assignments']:
            row['old_owner'] = owners[row['task_id']]
        decision['assign_new'] = assign_new
        decision['candidate_checks'] = dispatch['candidate_checks']
        decision['unassigned_tasks'] = dispatch['unassigned_tasks']
    else:
        output, decision = proposed, dispatch
    decision['scan_ordering'] = {'policy_version': 'X2-Scan-V01',
        'input_queues': original, 'retained_scan_queues': retained,
        'dispatch_output_queues': copy.deepcopy(proposed), 'final_scan_queues': copy.deepcopy(output),
        'protected_heads': protected, 'pre_dispatch_owners': owners,
        'shared_dispatch_reference': dispatch if ordered != proposed else None,
        'post_dispatch_resort': ordered != proposed,
        'dispatch_rule': 'unchanged greedy tail-distance/capacity/append-budget; final scan route budget rechecked',
        'failure_policy': 'no hidden fallback when final scan route is infeasible'}
    # Execution pops its live queue; the saved decision must retain its own copy.
    decision['output_queues'] = copy.deepcopy(output)
    return output, plans, decision
