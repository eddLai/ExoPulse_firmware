"""
Minimal gymnasium-compatible environment stub for depRL agent loading.

deprl.utils.load() requires an environment with observation_space and
action_space to initialize the agent's neural network dimensions.
This stub provides those spaces without any simulation logic.
"""

import numpy as np

try:
    from gymnasium.spaces import Box
except ImportError:
    from gym.spaces import Box


class DummyExoEnv:
    """Lightweight environment stub for agent initialization only.

    Observation space (125-dim) matches the full SCONE observation vector.
    Action space (20-dim) matches 18 human muscles + 2 exo actuators.

    These dimensions can be overridden via constructor args to match
    the specific checkpoint being loaded.
    """

    def __init__(self, obs_dim=125, action_dim=20):
        self.observation_space = Box(
            low=-np.inf, high=np.inf,
            shape=(obs_dim,), dtype=np.float32,
        )
        self.action_space = Box(
            low=0.0, high=1.0,
            shape=(action_dim,), dtype=np.float32,
        )
        # Required by deprl.utils.load() / tonic compatibility
        self.results_dir = ""
        self._max_episode_steps = 1000
        self.name = "ExoPulse_hardware"
