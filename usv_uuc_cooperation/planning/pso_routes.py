"""PSO-specific safe-prefix failure policy; physical failures remain separate."""
import copy
from usv_uuc_cooperation.planning.continuous_routes import rebuild


def pso_rebuild(*args, **kwargs):
    queues, plans, decision = rebuild(*args, **kwargs, allow_planning_holds=True)
    decision['output_queues'] = copy.deepcopy(queues)
    return queues, plans, decision
