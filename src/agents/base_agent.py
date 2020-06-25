import time

# import torch

# device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
# dtype = torch.double


class BaseAgent(object):
    def __init__(self, env):
        self.env = env
        self.policy = None

    def eval(self, episodes, verbose=False):
        """ Evaluates model performance """

        print("\nEvaluating model for " + str(episodes) + " episodes ...\n")
        start_time = time.time()
        if self.policy is not None:
            self.policy.eval()
        rewards = []

        for episode in range(episodes):

            observation = self.env.reset()
            # observation = torch.tensor(observation, device=device, dtype=dtype)
            done = False
            episode_rewards = []

            while not done:
                action = self._select_action(observation)
                next_observation, reward, done, _ = self.env.step(action)
                episode_rewards.append(float(reward))
                # next_observation = torch.tensor(
                #     next_observation, device=device, dtype=dtype
                # )
                observation = next_observation

            total_episode_reward = sum(episode_rewards)
            rewards.append(total_episode_reward)
            print("Episode " + str(episode + 1) + ": Total Episode Reward = " +
                  str(total_episode_reward))
            rewards.append(total_episode_reward)

        self.env.close()
        print("\nAverage Reward for an episode = " +
              str(sum(rewards) / len(rewards)))
        print("Evaluation Completed in " + str(time.time() - start_time) +
              " seconds")

    def _select_action(self, obs):
        raise NotImplementedError
