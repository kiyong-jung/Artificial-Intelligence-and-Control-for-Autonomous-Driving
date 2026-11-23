"""Baseline program: the rule expert (ExpertPolicy v4) of the Hapjeong repository.

It acts only on the 43-value observation: IDM car-following with virtual stop points at red stop lines,
service stops, occupied crosswalks and lane ends. It picks lanes from the per-lane route connectivity.
"""


def make_policy(env):
    from env.route_policy import make_expert
    return make_expert(env)
