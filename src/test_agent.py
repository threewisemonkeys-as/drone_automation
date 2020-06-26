import agents
import envs

env = envs.SimpleDepthMavEnv()
env.reset()
env.step(0)
env.step(1)
# agent = agents.RandomAgent(env)
# agent.eval(1)
