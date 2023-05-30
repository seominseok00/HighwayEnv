# Hide pygame support prompt
import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'


from gymnasium.envs.registration import register


def register_highway_envs():
    """Import the envs module so that envs register themselves."""

    # merge_env.py
    register(
        id='merge-v0',
        entry_point='highway_env.envs:MergeEnv',
    )

    register(
        id='merge-v1',
        entry_point='highway_env.envs:MergeEnv1',
    )

    register(
        id='merge-v2',
        entry_point='highway_env.envs:MergeEnv2',
    )