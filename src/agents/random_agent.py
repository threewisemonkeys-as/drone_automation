from .base_agent import BaseAgent


class RandomAgent(BaseAgent):
    def __init__(self, env):
        super(RandomAgent, self).__init__(env)

    def _select_action(self, obs):
        return self.env.action_space.sample()
